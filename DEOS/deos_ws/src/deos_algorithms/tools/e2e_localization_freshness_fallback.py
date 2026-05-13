#!/usr/bin/env python3
"""
ROS-less E2E: Localization freshness/fallback (Ek Mimari uyumu)

Amaç:
- /final_odom üretim mantığının beklenen şekilde "ICP taze ise onu seç, değilse EKF" çalıştığını
  PASS/FAIL ile doğrulamak.

Not:
Bu test ROS node'u ayağa kaldırmaz; seçim kuralını deterministik olarak test eder.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Callable


def _ensure_utf8_stdio() -> None:
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if stream is not None and hasattr(stream, "reconfigure"):
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except Exception:
                pass


def _print_output_key_legend() -> None:
    print("\n-- ÇIKTI ALANLARI SÖZLÜĞÜ --")
    print(" chosen_source : final_odom için seçilen kaynak (icp / ekf / none)")
    print(" icp_age_s     : ICP odom tazelik yaşı (saniye)")
    print(" icp_timeout_s : ICP taze kabul eşiği (saniye)")


def choose_final_odom_source(*, icp_present: bool, ekf_present: bool, icp_age_s: float, icp_timeout_s: float) -> str:
    icp_fresh = bool(icp_present) and (float(icp_age_s) <= float(icp_timeout_s))
    if icp_fresh:
        return "icp"
    if ekf_present:
        return "ekf"
    return "none"


def _case(
    *,
    name: str,
    desc: str,
    expected: dict[str, Any],
    actual: dict[str, Any],
    actions_if_pass: list[str] | None = None,
) -> bool:
    ok = True
    mism: dict[str, tuple[Any, Any]] = {}
    for k, ev in expected.items():
        av = actual.get(k)
        if callable(ev):
            try:
                passed = bool(ev(av))
            except Exception:
                passed = False
            if not passed:
                ok = False
                mism[k] = ("<koşul>", av)
        else:
            if av != ev:
                ok = False
                mism[k] = (ev, av)

    status = "GEÇTİ" if ok else "KALDI"
    print(f"\n[{status}] {name}")
    print(f"  senaryo  : {desc}")
    print("  beklenen :", json.dumps({k: ("<koşul>" if callable(v) else v) for k, v in expected.items()}, ensure_ascii=False))
    print("  çıkan    :", json.dumps(actual, ensure_ascii=False))
    if ok and actions_if_pass:
        print("  eylemler :")
        for a in actions_if_pass:
            print(f"   - {a}")
    if mism:
        print("  farklar  :")
        for k, (ev, av) in mism.items():
            print(f"   - {k}: beklenen={ev} çıkan={av}")
    return ok


def main() -> int:
    _ensure_utf8_stdio()
    print("== E2E: Localization freshness/fallback (Ek Mimari) ==")
    _print_output_key_legend()

    total = 0
    passed = 0

    icp_timeout_s = 0.2

    # 1) ICP fresh -> choose icp
    actual = {
        "chosen_source": choose_final_odom_source(icp_present=True, ekf_present=True, icp_age_s=0.05, icp_timeout_s=icp_timeout_s),
        "icp_age_s": 0.05,
        "icp_timeout_s": icp_timeout_s,
    }
    total += 1
    passed += _case(
        name="ICP taze iken /final_odom ICP seçer",
        desc="Ek mimari: LiDAR scan matching tazeyse /final_odom ICP bazlı olmalı.",
        expected={"chosen_source": "icp"},
        actual=actual,
        actions_if_pass=["Planning/heading kaynağı ICP tabanlı /final_odom üzerinden stabilize olur."],
    )

    # 2) ICP stale -> choose ekf
    actual = {
        "chosen_source": choose_final_odom_source(icp_present=True, ekf_present=True, icp_age_s=1.0, icp_timeout_s=icp_timeout_s),
        "icp_age_s": 1.0,
        "icp_timeout_s": icp_timeout_s,
    }
    total += 1
    passed += _case(
        name="ICP bayat iken /final_odom EKF seçer",
        desc="ICP gelmezse veya bayatlarsa sistem global stabilite için EKF /odom'a düşmeli.",
        expected={"chosen_source": "ekf"},
        actual=actual,
        actions_if_pass=["/final_odom EKF /odom ile devam eder (ICP kesintisinde araç 'kör' kalmaz)."],
    )

    # 3) ICP missing -> choose ekf
    actual = {
        "chosen_source": choose_final_odom_source(icp_present=False, ekf_present=True, icp_age_s=999.0, icp_timeout_s=icp_timeout_s),
        "icp_age_s": 999.0,
        "icp_timeout_s": icp_timeout_s,
    }
    total += 1
    passed += _case(
        name="ICP yoksa /final_odom EKF seçer",
        desc="PCL localization çalışmıyorsa sistem EKF ile çalışabilmeli.",
        expected={"chosen_source": "ekf"},
        actual=actual,
        actions_if_pass=["PCL localization devre dışı kalsa bile /final_odom ile temel sürüş mümkün."],
    )

    # 4) None -> none
    actual = {
        "chosen_source": choose_final_odom_source(icp_present=False, ekf_present=False, icp_age_s=999.0, icp_timeout_s=icp_timeout_s),
        "icp_age_s": 999.0,
        "icp_timeout_s": icp_timeout_s,
    }
    total += 1
    passed += _case(
        name="ICP+EKF yoksa /final_odom üretilemez",
        desc="Her iki kaynak da yoksa sistem lokalizasyon verisi yayınlayamaz.",
        expected={"chosen_source": "none"},
        actual=actual,
        actions_if_pass=["Fail-safe: planning hız düşürme/durma politikasına girmeli (entegrasyon gerektirir)."],
    )

    print(f"\n== SONUÇ: {passed}/{total} geçti ==")
    return 0 if passed == total else 2


if __name__ == "__main__":
    raise SystemExit(main())

