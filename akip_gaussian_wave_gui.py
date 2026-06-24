#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
akip_gaussian_wave_gui.py

Отдельная программа для АКИП-3407/1А:
- кнопка инициализации АКИП;
- настройка длительности GAUS-волны;
- настройка максимального уровня;
- кнопка запуска одной гауссовой волны (*TRG);
- аварийное выключение OUTP/BURS.

Положите CH375DLL64.dll рядом с этим .py файлом или укажите путь к DLL в поле программы.
Запускать на Windows той же разрядности, что DLL и Python.
"""

from __future__ import annotations

import ctypes
import math
import threading
import time
from ctypes import byref, create_string_buffer, wintypes
from pathlib import Path
from typing import Optional

import tkinter as tk
from tkinter import filedialog, messagebox, ttk


class AkipGaussianWaveController:
    """Управление одной GAUS-волной АКИП через CH375DLL64.dll.

    Последовательность команд сохранена по рабочей логике из final_bci_three_modes:
        OUTP OFF -> BURS OFF -> FUNC GAUSsian -> FREQ/PER -> VOLT:LOW/HIGH
        -> BURS:MODE TRIG -> BURS:NCYC 1 -> BURS:STAT ON -> TRIG:SOUR EXT
        -> OUTP ON.

    После этого одна команда *TRG выдаёт одну гауссову волну.
    """

    TERMINATOR = b"\r\n"
    DEVICE_INDEX = 0

    MIN_HIGH_MV = 0.2          # около 0.2 мВ для высокоомной нагрузки
    MAX_HIGH_MV = 20_000.0     # 20 В = 20000 мВ

    def __init__(self) -> None:
        self.dll = None
        self.handle = None
        self.read_function = None
        self.ready = False
        self.failed_reason = ""
        self.lock = threading.Lock()
        self.trigger_count = 0
        self.last_trigger_perf: Optional[float] = None

        self.dll_path: Optional[Path] = None
        self.wave_period_ms = 200.0
        self.high_mv = 200.0

    @property
    def wave_frequency_hz(self) -> float:
        return 1000.0 / max(self.wave_period_ms, 1e-9)

    def configure(self, *, wave_period_ms: float, high_mv: float, dll_path: Optional[str]) -> None:
        wave_period_ms = float(wave_period_ms)
        high_mv = float(high_mv)

        if not math.isfinite(wave_period_ms) or not (1.0 <= wave_period_ms <= 300.0):
            raise ValueError("Длительность GAUS должна быть от 1 до 300 мс.")
        if not math.isfinite(high_mv) or not (self.MIN_HIGH_MV <= high_mv <= self.MAX_HIGH_MV):
            raise ValueError("Максимальный уровень АКИП должен быть от 0.2 мВ до 20000 мВ.")

        with self.lock:
            if self.ready:
                raise RuntimeError("АКИП уже инициализирован. Сначала нажмите «Выключить выход», затем инициализируйте заново.")
            self.wave_period_ms = wave_period_ms
            self.high_mv = high_mv
            self.dll_path = Path(dll_path).expanduser() if dll_path else None

    def _find_dll_path(self) -> str:
        candidates = []
        if self.dll_path:
            candidates.append(self.dll_path)
        candidates.extend([
            Path(__file__).with_name("CH375DLL64.dll"),
            Path.cwd() / "CH375DLL64.dll",
        ])
        for path in candidates:
            if path.exists():
                return str(path)
        raise FileNotFoundError("Не найден CH375DLL64.dll. Положите DLL рядом с программой или укажите путь к DLL.")

    def _load_dll(self) -> None:
        if self.dll is not None:
            return
        if not hasattr(ctypes, "WinDLL"):
            raise RuntimeError("Управление АКИП через CH375DLL64.dll доступно только на Windows.")

        self.dll = ctypes.WinDLL(self._find_dll_path())

        self.dll.CH375OpenDevice.argtypes = [wintypes.ULONG]
        self.dll.CH375OpenDevice.restype = wintypes.HANDLE

        self.dll.CH375CloseDevice.argtypes = [wintypes.ULONG]
        self.dll.CH375CloseDevice.restype = None

        self.dll.CH375SetTimeout.argtypes = [wintypes.ULONG, wintypes.ULONG, wintypes.ULONG]
        self.dll.CH375SetTimeout.restype = wintypes.BOOL

        self.dll.CH375WriteData.argtypes = [wintypes.ULONG, ctypes.c_void_p, ctypes.POINTER(wintypes.ULONG)]
        self.dll.CH375WriteData.restype = wintypes.BOOL

        self.read_function = getattr(self.dll, "CH375ReadData", None)
        if self.read_function is not None:
            self.read_function.argtypes = [wintypes.ULONG, ctypes.c_void_p, ctypes.POINTER(wintypes.ULONG)]
            self.read_function.restype = wintypes.BOOL

    def _send_locked(self, command: str, delay: float = 0.0, verbose: bool = True) -> None:
        if self.dll is None:
            raise RuntimeError("CH375DLL64.dll не загружена.")

        data = command.encode("ascii") + self.TERMINATOR
        length = wintypes.ULONG(len(data))
        buffer = create_string_buffer(data)
        ok = self.dll.CH375WriteData(self.DEVICE_INDEX, buffer, byref(length))

        if verbose:
            status = "OK" if ok and length.value == len(data) else "ERROR"
            print(f"[AKIP {status}] {command} | sent={length.value}/{len(data)}")

        if not ok or length.value != len(data):
            raise RuntimeError(f"Не удалось передать команду в АКИП: {command}")
        if delay > 0:
            time.sleep(delay)

    def _query_locked(self, command: str, wait_sec: float = 0.12) -> Optional[str]:
        if self.read_function is None:
            return None
        self._send_locked(command, delay=wait_sec, verbose=False)
        buffer = create_string_buffer(256)
        length = wintypes.ULONG(256)
        ok = self.read_function(self.DEVICE_INDEX, buffer, byref(length))
        if not ok or length.value == 0:
            return None
        return buffer.raw[:length.value].decode("ascii", errors="replace").strip("\x00\r\n ")

    @staticmethod
    def _norm_reply(value: Optional[str]) -> str:
        return str(value or "").strip().upper()

    def setup(self) -> bool:
        """Инициализировать АКИП и оставить его готовым к одиночным *TRG."""
        with self.lock:
            if self.ready:
                return True
            try:
                self._load_dll()

                self.handle = self.dll.CH375OpenDevice(self.DEVICE_INDEX)
                if not self.handle:
                    raise RuntimeError("Не удалось открыть АКИП-3407/1А через CH375.")

                self.dll.CH375SetTimeout(self.DEVICE_INDEX, 3000, 3000)

                self._send_locked("OUTP OFF", delay=0.25)
                self._send_locked("BURS:STAT OFF", delay=0.20)

                self._send_locked("FUNC GAUSsian", delay=0.20)
                self._send_locked(f"FREQ {self.wave_frequency_hz:.9f}Hz", delay=0.20)
                self._send_locked(f"PER {self.wave_period_ms:.9g}ms", delay=0.20)
                self._send_locked("VOLT:LOW 0mV", delay=0.20)
                self._send_locked(f"VOLT:HIGH {self.high_mv:.9g}mV", delay=0.20)

                self._send_locked("BURS:MODE TRIG", delay=0.20)
                self._send_locked("BURS:NCYC 1", delay=0.20)
                self._send_locked("BURS:PHAS 0deg", delay=0.20)
                self._send_locked("BURS:STAT ON", delay=0.25)
                self._send_locked("TRIG:SOUR EXT", delay=0.25)

                # Повтор критических параметров — как в рабочей программе.
                self._send_locked("BURS:MODE TRIG", delay=0.15)
                self._send_locked("BURS:NCYC 1", delay=0.15)
                self._send_locked("BURS:PHAS 0deg", delay=0.15)
                self._send_locked(f"FREQ {self.wave_frequency_hz:.9f}Hz", delay=0.15)
                self._send_locked(f"PER {self.wave_period_ms:.9g}ms", delay=0.15)
                self._send_locked("TRIG:SOUR EXT", delay=0.20)

                trigger_source = self._norm_reply(self._query_locked("TRIG:SOUR?"))
                burst_state = self._norm_reply(self._query_locked("BURS:STAT?"))
                burst_cycles = self._norm_reply(self._query_locked("BURS:NCYC?"))

                if trigger_source and trigger_source != "EXT":
                    raise RuntimeError(f"АКИП не перешёл в EXT: TRIG:SOUR?={trigger_source!r}")
                if burst_state and burst_state not in {"1", "ON"}:
                    raise RuntimeError(f"Burst не включён: BURS:STAT?={burst_state!r}")
                if burst_cycles:
                    try:
                        cycles = float(burst_cycles)
                    except ValueError:
                        cycles = 1.0
                    if abs(cycles - 1.0) > 1e-9:
                        raise RuntimeError(f"BURS:NCYC не равен 1: {burst_cycles!r}")

                self._send_locked("OUTP ON", delay=0.30)

                self.ready = True
                self.failed_reason = ""
                self.trigger_count = 0
                self.last_trigger_perf = None
                print(f"[AKIP] Готов: GAUS {self.wave_period_ms:g} мс, 0...{self.high_mv:g} мВ, EXT + *TRG.")
                return True

            except Exception as exc:
                self.ready = False
                self.failed_reason = str(exc)
                try:
                    if self.dll is not None and self.handle:
                        try:
                            self._send_locked("BURS:STAT OFF", delay=0.10, verbose=False)
                        except Exception:
                            pass
                        try:
                            self._send_locked("OUTP OFF", delay=0.10, verbose=False)
                        except Exception:
                            pass
                        self.dll.CH375CloseDevice(self.DEVICE_INDEX)
                except Exception:
                    pass
                self.handle = None
                print(f"[AKIP] Ошибка инициализации; Burst и выход выключены: {exc}")
                return False

    def trigger_one_wave(self) -> bool:
        """Одна команда *TRG — одна GAUS-волна."""
        with self.lock:
            if not self.ready:
                print(f"[AKIP] Волна пропущена: генератор не готов. {self.failed_reason}")
                return False
            try:
                self._send_locked("*TRG", delay=0.0, verbose=False)
                self.trigger_count += 1
                self.last_trigger_perf = time.perf_counter()
                print(f"[AKIP] *TRG #{self.trigger_count}: один пакет GAUS.")
                return True
            except Exception as exc:
                self.failed_reason = str(exc)
                print(f"[AKIP] Ошибка запуска GAUS: {exc}")
                return False

    def output_off(self) -> None:
        """Выключить Burst и физический выход."""
        with self.lock:
            if self.dll is None or not self.handle:
                self.ready = False
                return
            try:
                self._send_locked("BURS:STAT OFF", delay=0.05, verbose=False)
            except Exception:
                pass
            try:
                self._send_locked("OUTP OFF", delay=0.05, verbose=False)
            except Exception:
                pass
            self.ready = False
            print("[AKIP] BURS OFF, OUTP OFF.")

    def close(self) -> None:
        with self.lock:
            try:
                if self.dll is not None:
                    try:
                        self._send_locked("BURS:STAT OFF", delay=0.05, verbose=False)
                        self._send_locked("OUTP OFF", delay=0.05, verbose=False)
                    except Exception:
                        pass
                    if self.handle:
                        self.dll.CH375CloseDevice(self.DEVICE_INDEX)
            except Exception:
                pass
            self.ready = False
            self.handle = None


class AkipGaussianWaveApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("АКИП — одиночная гауссова волна")
        self.geometry("760x560")
        self.minsize(680, 500)

        self.ctrl = AkipGaussianWaveController()
        self.setup_pending = False
        self.setup_result: Optional[bool] = None

        self.dll_path_var = tk.StringVar(value=str(Path(__file__).with_name("CH375DLL64.dll")))
        self.duration_ms_var = tk.StringVar(value="200")
        self.mode_var = tk.StringVar(value="divider")
        self.eeg_peak_uv_var = tk.StringVar(value="20")
        self.divider_ratio_var = tk.StringVar(value="10000")
        self.direct_high_mv_var = tk.StringVar(value="200")
        self.calculated_high_mv_var = tk.StringVar(value="200.000")
        self.status_var = tk.StringVar(value="АКИП не инициализирован.")
        self.info_var = tk.StringVar(value="")

        self._build_ui()
        self._recalculate_level()
        self.after(100, self._poll_setup_result)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=14)
        root.pack(fill="both", expand=True)

        title = ttk.Label(root, text="АКИП-3407/1А: одна GAUS-волна по кнопке", font=("Segoe UI", 16, "bold"))
        title.pack(anchor="w", pady=(0, 10))

        dll_box = ttk.LabelFrame(root, text="DLL")
        dll_box.pack(fill="x", pady=(0, 10))
        dll_row = ttk.Frame(dll_box, padding=8)
        dll_row.pack(fill="x")
        ttk.Entry(dll_row, textvariable=self.dll_path_var).pack(side="left", fill="x", expand=True)
        ttk.Button(dll_row, text="Выбрать...", command=self._choose_dll).pack(side="left", padx=(8, 0))

        settings = ttk.LabelFrame(root, text="Настройки волны")
        settings.pack(fill="x", pady=(0, 10))
        grid = ttk.Frame(settings, padding=10)
        grid.pack(fill="x")

        ttk.Label(grid, text="Длительность GAUS, мс:").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(grid, textvariable=self.duration_ms_var, width=14).grid(row=0, column=1, sticky="w", pady=4)
        ttk.Label(grid, text="1…300 мс; частота задаётся как 1000 / длительность").grid(row=0, column=2, sticky="w", padx=(10, 0), pady=4)

        ttk.Radiobutton(grid, text="Задать через пик на EEG и делитель", variable=self.mode_var, value="divider", command=self._recalculate_level).grid(row=1, column=0, columnspan=3, sticky="w", pady=(10, 2))
        ttk.Label(grid, text="Желаемый пик на EEG, мкВ:").grid(row=2, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(grid, textvariable=self.eeg_peak_uv_var, width=14).grid(row=2, column=1, sticky="w", pady=4)
        ttk.Label(grid, text="Коэффициент делителя:").grid(row=3, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(grid, textvariable=self.divider_ratio_var, width=14).grid(row=3, column=1, sticky="w", pady=4)

        ttk.Radiobutton(grid, text="Задать максимум АКИП напрямую", variable=self.mode_var, value="direct", command=self._recalculate_level).grid(row=4, column=0, columnspan=3, sticky="w", pady=(10, 2))
        ttk.Label(grid, text="VOLT:HIGH, мВ:").grid(row=5, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(grid, textvariable=self.direct_high_mv_var, width=14).grid(row=5, column=1, sticky="w", pady=4)

        ttk.Label(grid, text="Рассчитанный максимум АКИП, мВ:").grid(row=6, column=0, sticky="w", padx=(0, 8), pady=(10, 4))
        ttk.Label(grid, textvariable=self.calculated_high_mv_var, font=("Segoe UI", 10, "bold")).grid(row=6, column=1, sticky="w", pady=(10, 4))
        ttk.Button(grid, text="Пересчитать", command=self._recalculate_level).grid(row=6, column=2, sticky="w", padx=(10, 0), pady=(10, 4))
        grid.columnconfigure(2, weight=1)

        buttons = ttk.Frame(root)
        buttons.pack(fill="x", pady=(4, 10))
        self.init_button = ttk.Button(buttons, text="Инициализировать АКИП", command=self.initialize_akip)
        self.init_button.pack(side="left", padx=(0, 8))
        self.wave_button = ttk.Button(buttons, text="Дать одну GAUS-волну", command=self.send_one_wave)
        self.wave_button.pack(side="left", padx=(0, 8))
        ttk.Button(buttons, text="Выключить выход", command=self.output_off).pack(side="left", padx=(0, 8))

        status_box = ttk.LabelFrame(root, text="Статус")
        status_box.pack(fill="both", expand=True)
        ttk.Label(status_box, textvariable=self.status_var, wraplength=690, foreground="#0f172a", padding=8).pack(anchor="w", fill="x")
        ttk.Label(status_box, textvariable=self.info_var, wraplength=690, foreground="#475569", padding=8).pack(anchor="w", fill="x")

        note = (
            "Важно: до подключения к EEG используйте делитель/изоляцию и проверяйте уровень осциллографом. "
            "Кнопка волны отправляет только *TRG; форма, длительность и уровень задаются при инициализации."
        )
        ttk.Label(root, text=note, wraplength=710, foreground="#7c2d12").pack(anchor="w", pady=(8, 0))

        for var in (self.duration_ms_var, self.eeg_peak_uv_var, self.divider_ratio_var, self.direct_high_mv_var):
            var.trace_add("write", lambda *_: self._safe_recalculate())

    def _choose_dll(self) -> None:
        path = filedialog.askopenfilename(title="Выберите CH375DLL64.dll", filetypes=[("DLL", "*.dll"), ("All files", "*.*")])
        if path:
            self.dll_path_var.set(path)

    def _safe_recalculate(self) -> None:
        try:
            self._recalculate_level(show_errors=False)
        except Exception:
            pass

    def _recalculate_level(self, show_errors: bool = True) -> float:
        try:
            if self.mode_var.get() == "divider":
                eeg_peak_uv = float(self.eeg_peak_uv_var.get().replace(",", "."))
                divider = float(self.divider_ratio_var.get().replace(",", "."))
                high_mv = eeg_peak_uv * divider / 1000.0
            else:
                high_mv = float(self.direct_high_mv_var.get().replace(",", "."))
            self.calculated_high_mv_var.set(f"{high_mv:.6g}")
            duration_ms = float(self.duration_ms_var.get().replace(",", "."))
            freq = 1000.0 / max(duration_ms, 1e-9)
            self.info_var.set(f"Будет настроено: FUNC GAUSsian, PER {duration_ms:g} ms, FREQ {freq:.6g} Hz, VOLT:LOW 0 mV, VOLT:HIGH {high_mv:.6g} mV, BURS:NCYC 1, TRIG:SOUR EXT.")
            return high_mv
        except Exception as exc:
            if show_errors:
                messagebox.showerror("Ошибка настроек", str(exc))
            raise

    def _read_settings(self) -> tuple[float, float, str]:
        duration_ms = float(self.duration_ms_var.get().replace(",", "."))
        high_mv = self._recalculate_level(show_errors=True)
        dll_path = self.dll_path_var.get().strip()
        return duration_ms, high_mv, dll_path

    def initialize_akip(self) -> None:
        if self.setup_pending:
            return
        try:
            duration_ms, high_mv, dll_path = self._read_settings()
            self.ctrl.configure(wave_period_ms=duration_ms, high_mv=high_mv, dll_path=dll_path)
        except Exception as exc:
            messagebox.showerror("АКИП", str(exc))
            return

        self.setup_pending = True
        self.setup_result = None
        self.status_var.set("АКИП: выполняется инициализация...")
        self.init_button.state(["disabled"])

        def worker() -> None:
            self.setup_result = self.ctrl.setup()

        threading.Thread(target=worker, daemon=True, name="AKIP-GAUS-setup").start()

    def _poll_setup_result(self) -> None:
        if self.setup_pending and self.setup_result is not None:
            ok = bool(self.setup_result)
            self.setup_pending = False
            self.init_button.state(["!disabled"])
            if ok:
                self.status_var.set(f"АКИП готов. Нажмите «Дать одну GAUS-волну». Счётчик волн: {self.ctrl.trigger_count}.")
            else:
                self.status_var.set(f"Ошибка инициализации: {self.ctrl.failed_reason}")
        self.after(100, self._poll_setup_result)

    def send_one_wave(self) -> None:
        if not self.ctrl.ready:
            messagebox.showwarning("АКИП", "Сначала нажмите «Инициализировать АКИП».")
            return
        ok = self.ctrl.trigger_one_wave()
        if ok:
            self.status_var.set(f"GAUS-волна отправлена (*TRG). Счётчик волн: {self.ctrl.trigger_count}.")
        else:
            self.status_var.set(f"Ошибка запуска волны: {self.ctrl.failed_reason}")

    def output_off(self) -> None:
        self.ctrl.output_off()
        self.status_var.set("Выход выключен: BURS OFF, OUTP OFF. Для новой волны сначала инициализируйте АКИП.")

    def _on_close(self) -> None:
        try:
            self.ctrl.close()
        finally:
            self.destroy()


if __name__ == "__main__":
    app = AkipGaussianWaveApp()
    app.mainloop()
