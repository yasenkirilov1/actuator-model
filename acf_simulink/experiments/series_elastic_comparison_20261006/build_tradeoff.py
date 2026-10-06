#!/usr/bin/env python3
"""Bounded static arithmetic for a leadscrew with actuator-internal elasticity.

This does not run a dynamic model, simulate a controller, or qualify contact force.
It keeps seven rows plus scalar grid extrema, with no MAT/NPZ/native histories.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]
EXPERIMENT = Path(__file__).resolve().parent
VALIDATION = ROOT / "acf_simulink/validation/series_elastic_comparison_20261006"
FIGURE = ROOT / "reports/figures/series-elastic-leadscrew-tradeoff-20261006.svg"
RATES_N_PER_MM = (0.5, 1.0, 2.0, 3.5, 5.0, 10.0, 20.0)
G = 9.80665
A = 1.5
B = 0.6
S_RANGE = (0.1, 2.0)
FORCE_RANGE = (5.0, 50.0)
K_UPPER = 20_000.0
K_CELL = 64_000_000.0
LAG_MM = 0.1
REFERENCE_STROKE_MM = 25.0
ARRAY_BUDGET_BYTES = 5 * 1024**3
RETAINED_FORECAST_BYTES = 250_000
TEMPORARY_FORECAST_BYTES = 1_000_000


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")


def check_storage() -> dict:
    """Count existing project evidence arrays, excluding symlink aliases."""
    scopes = (
        ROOT / "acf_simulink/validation",
        Path("/run/media/tommie/SATA SSD/actuator-model-evidence"),
    )
    seen = set()
    total = 0
    count = 0
    for scope in scopes:
        if not scope.exists():
            continue
        for dirname, _, names in os.walk(scope, followlinks=False):
            for name in names:
                if not name.lower().endswith((".mat", ".npz")):
                    continue
                path = Path(dirname) / name
                if path.is_symlink():
                    continue
                stat = path.stat()
                identity = (stat.st_dev, stat.st_ino)
                if identity in seen:
                    continue
                seen.add(identity)
                total += stat.st_size
                count += 1
    free = shutil.disk_usage(ROOT).free
    temporary_free = shutil.disk_usage("/tmp").free
    assert total <= ARRAY_BUDGET_BYTES, (total, ARRAY_BUDGET_BYTES)
    assert free > RETAINED_FORECAST_BYTES
    assert temporary_free > TEMPORARY_FORECAST_BYTES
    return {
        "scopes": [str(p) for p in scopes],
        "current_unique_mat_npz_files": count,
        "current_unique_mat_npz_bytes": total,
        "shared_array_budget_bytes": ARRAY_BUDGET_BYTES,
        "new_array_bytes": 0,
        "retained_new_source_metrics_figure_forecast_bytes": RETAINED_FORECAST_BYTES,
        "temporary_forecast_bytes": TEMPORARY_FORECAST_BYTES,
        "project_filesystem_free_bytes_before": free,
        "temporary_filesystem_free_bytes_before": temporary_free,
        "simulation_or_training_launched": False,
        "method": "Stream scalar static-equilibrium calculations; retain seven metrics rows.",
    }


def q_formula(force: float, tool_mass: float, cosine: float, k_sea: float) -> float:
    g_axis = G * cosine
    return (
        (force - (A + B + tool_mass) * g_axis) / K_UPPER
        + (force - (B + tool_mass) * g_axis) / k_sea
        + (force - tool_mass * g_axis) / K_CELL
    )


def dense_grid_coordinate_extrema(k_sea: float) -> dict:
    """Independent construction from three static force balances and positions.

    Set robot and tool positions to zero. The load cell balance first determines
    B, the upper spring balance determines A, and the series spring locates nut N.
    q is N relative to A. Only scalar extrema are retained.
    """
    q_min = math.inf
    q_max = -math.inf
    min_point = None
    max_point = None
    points = 0
    for force_index in range(91):
        contact_force = 5.0 + 0.5 * force_index
        for mass_index in range(39):
            m_s = 0.1 + 0.05 * mass_index
            for angle_index in range(181):
                theta = 0.5 * angle_index
                g_axis = G * math.cos(math.radians(theta))
                f_cell = contact_force - m_s * g_axis
                f_series = f_cell - B * g_axis
                f_upper = f_series - A * g_axis
                x_s = 0.0
                x_r = 0.0
                x_b = x_s + f_cell / K_CELL
                x_a = x_r - f_upper / K_UPPER
                x_n = x_b + f_series / k_sea
                q = x_n - x_a
                assert math.isclose(f_cell + m_s * g_axis, contact_force, abs_tol=1e-12)
                assert math.isclose(f_series + B * g_axis, f_cell, abs_tol=1e-12)
                assert math.isclose(f_upper + A * g_axis, f_series, abs_tol=1e-12)
                if q < q_min:
                    q_min = q
                    min_point = {"force_N": contact_force, "S_kg": m_s, "theta_deg": theta}
                if q > q_max:
                    q_max = q
                    max_point = {"force_N": contact_force, "S_kg": m_s, "theta_deg": theta}
                points += 1
    return {
        "points": points,
        "nut_position_min_mm": 1000 * q_min,
        "nut_position_max_mm": 1000 * q_max,
        "nut_position_span_mm": 1000 * (q_max - q_min),
        "one_min_point": min_point,
        "one_max_point": max_point,
    }


def figure(rows: list[dict], threshold: float) -> str:
    """Simple vector schematic plus two small charts; no raster dependency."""
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="900" height="780" viewBox="0 0 900 780" role="img" aria-labelledby="title desc">',
        '<title id="title">A spring-coupled leadscrew trades force sensitivity for nut travel</title>',
        '<desc id="desc">The added spring is between screw nut and B, separate from the prescribed upper spring and B-S sensor. Graphs show arithmetic spring-force change for 0.1 millimetre relative lag and static nut-travel span across all stated force, gravity orientation and tool-mass endpoints. No controller performance is simulated.</desc>',
        '<defs><marker id="arrow" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto"><path d="M0 0 L7 3.5 L0 7" fill="#374151"/></marker></defs>',
        '<style>text{font-family:Arial,sans-serif;fill:#18263b} .small{font-size:12px} .body{font-size:14px} .label{font-size:13px} .axis{stroke:#7b8799;stroke-width:1} .grid{stroke:#e0e6ec;stroke-width:1} .blue{stroke:#176ba0;fill:none;stroke-width:3} .teal{stroke:#00866c;fill:none;stroke-width:3}</style>',
        '<rect width="900" height="780" rx="12" fill="#ffffff"/>',
        '<text x="40" y="35" font-size="24" font-weight="700">A leadscrew can control force through a spring</text>',
        '<text x="40" y="60" class="body">The spring belongs inside the actuator, between its nut and holder B.</text>',
        '<rect x="39" y="85" width="822" height="212" rx="9" fill="#f4f8fb" stroke="#d4e0e9"/>',
        '<text x="57" y="110" class="label" font-weight="700">Prescribed upper spring</text>',
        '<text x="57" y="127" class="small">20 N/mm · zero damping</text>',
        '<rect x="299" y="118" width="240" height="85" rx="7" fill="#e9f4f2" stroke="#70b7a8"/>',
        '<text x="419" y="136" class="small" text-anchor="middle">Actuator with an internal series spring</text>',
        '<line x1="84" y1="164" x2="802" y2="164" stroke="#61768a" stroke-width="2"/>',
        '<rect x="57" y="145" width="46" height="38" rx="4" fill="#fff" stroke="#65758a"/><text x="80" y="170" class="body" text-anchor="middle">R</text>',
        '<path d="M103 164 L120 164 L126 153 L138 175 L150 153 L162 175 L174 153 L186 175 L192 164 L214 164" fill="none" stroke="#176ba0" stroke-width="2.5"/>',
        '<rect x="214" y="145" width="47" height="38" rx="4" fill="#fff" stroke="#65758a"/><text x="237" y="170" class="body" text-anchor="middle">A</text>',
        '<rect x="310" y="145" width="92" height="38" rx="4" fill="#fff" stroke="#65758a"/><text x="356" y="162" class="small" text-anchor="middle">motor + screw</text><text x="356" y="177" class="small" text-anchor="middle">fixed to A</text>',
        '<rect x="411" y="145" width="43" height="38" rx="4" fill="#fff" stroke="#65758a"/><text x="432" y="170" class="label" text-anchor="middle">nut</text>',
        '<path d="M454 164 L462 164 L468 153 L480 175 L492 153 L504 175 L516 153 L522 164 L550 164" fill="none" stroke="#00866c" stroke-width="2.5"/>',
        '<rect x="550" y="145" width="44" height="38" rx="4" fill="#fff" stroke="#65758a"/><text x="572" y="170" class="body" text-anchor="middle">B</text>',
        '<rect x="616" y="145" width="77" height="38" rx="4" fill="#fff" stroke="#65758a"/><text x="654" y="170" class="label" text-anchor="middle">load cell</text>',
        '<rect x="713" y="145" width="44" height="38" rx="4" fill="#fff" stroke="#65758a"/><text x="735" y="170" class="body" text-anchor="middle">S</text>',
        '<line x1="802" y1="141" x2="802" y2="187" stroke="#374151" stroke-width="4"/>',
        '<text x="808" y="157" class="small">surface</text>',
        '<text x="421" y="223" class="body" text-anchor="middle" font-weight="700">Spring force = preload + stiffness × (nut position − B position)</text>',
        '<text x="58" y="255" class="label">Motor inertia J / ℓ² acts on the nut coordinate; the spring couples that motion to B.</text>',
        '<text x="58" y="277" class="label">ℓ = screw lead / (2π). B and S are separate from the nut, with finite cell stiffness.</text>',
        '<text x="40" y="333" font-size="18" font-weight="700">Softer springs reduce sensitivity, but require more travel</text>',
        '<text x="40" y="354" class="body">Static arithmetic · no damping · robot and surface positions fixed · no controller simulation</text>',
    ]

    def x_at(rate: float, left: float, width: float) -> float:
        return left + width * math.log(rate / 0.5) / math.log(40.0)

    for panel, (left, width, ymax, title, quantity, unit, color, yticks) in enumerate([
        (90, 316, 2.2, "Spring-force change for 0.1 mm lag", "spring_force_change_for_0_1mm_lag_N", "N", "#176ba0", (0, 0.5, 1.0, 1.5, 2.0)),
        (535, 315, 160.0, "Required static nut-position span", "nut_position_span_mm", "mm", "#00866c", (0, 25, 50, 100, 150)),
    ]):
        top, bottom = 406, 644
        svg.append(f'<text x="{left + width / 2}" y="386" font-size="15" font-weight="700" text-anchor="middle">{title}</text>')
        for tick in yticks:
            y = bottom - (bottom - top) * tick / ymax
            svg.append(f'<line x1="{left}" y1="{y:.3f}" x2="{left + width}" y2="{y:.3f}" class="grid"/>')
            svg.append(f'<text x="{left - 9}" y="{y + 4:.3f}" class="small" text-anchor="end">{tick:g}</text>')
        svg.append(f'<text x="{left - 26}" y="{top - 3}" class="label">{unit}</text>')
        svg.append(f'<line x1="{left}" y1="{top}" x2="{left}" y2="{bottom}" class="axis"/>')
        svg.append(f'<line x1="{left}" y1="{bottom}" x2="{left + width}" y2="{bottom}" class="axis"/>')
        polyline = []
        for index, row in enumerate(rows):
            x = x_at(row["spring_rate_N_per_mm"], left, width)
            y = bottom - (bottom - top) * row[quantity] / ymax
            polyline.append(f"{x:.3f},{y:.3f}")
            svg.append(f'<line x1="{x:.3f}" y1="{bottom}" x2="{x:.3f}" y2="{bottom + 5}" class="axis"/>')
            svg.append(f'<text x="{x:.3f}" y="{bottom + 21}" class="small" text-anchor="middle">{row["spring_rate_N_per_mm"]:g}</text>')
            svg.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="4" fill="{color}"/>')
            # Stagger point labels to avoid overlap near the low end of panel 2.
            label_y = y - 10 if panel == 0 or index <= 2 else y - (26 if index % 2 else 10)
            value = f'{row[quantity]:g}' if panel == 0 else f'{row[quantity]:.1f}'
            svg.append(f'<text x="{x:.3f}" y="{label_y:.3f}" class="small" text-anchor="middle" fill="{color}">{value}</text>')
        svg.append(f'<polyline points="{" ".join(polyline)}" fill="none" stroke="{color}" stroke-width="2.3"/>')
        svg.append(f'<text x="{left + width / 2}" y="692" class="label" text-anchor="middle">Spring rate (N/mm; logarithmic spacing)</text>')
        if panel == 1:
            y = bottom - (bottom - top) * REFERENCE_STROKE_MM / ymax
            svg.append(f'<line x1="{left}" y1="{y:.3f}" x2="{left + width}" y2="{y:.3f}" stroke="#a05a1a" stroke-width="1.5" stroke-dasharray="5 4"/>')
            svg.append(f'<text x="{left + 4}" y="{y - 6:.3f}" class="small" fill="#a05a1a">25 mm reference</text>')
    svg.extend([
        '<text x="40" y="727" class="label">Span covers 5–50 N, S = 0.1–2 kg, and gravity angles 0–90°; A = 1.5 kg, B = 0.6 kg.</text>',
        '<text x="40" y="748" class="label">Lag sensitivity is spring-force arithmetic, not measured contact error. A screw may use more than 25 mm.</text>',
        f'<text x="40" y="769" class="small">A fixed preload shifts the zero, not the span. This 25 mm static envelope needs k ≥ {threshold:.2f} N/mm; dynamics still need testing.</text>',
        '</svg>',
    ])
    result = "\n".join(svg) + "\n"
    ET.fromstring(result)
    return result


def main() -> None:
    VALIDATION.mkdir(parents=True, exist_ok=True)
    # Forecast and space/budget checks precede the bounded arithmetic work.
    forecast = check_storage()
    write_json(VALIDATION / "storage_forecast.json", forecast)
    rows = []
    force_series_min = 5.0 - (B + 2.0) * G
    force_series_max = 50.0
    force_series_span = force_series_max - force_series_min
    fixed_other_span_mm = 1000 * (
        (45.0 + (A + B + 2.0) * G) / K_UPPER
        + (45.0 + 2.0 * G) / K_CELL
    )
    threshold_n_per_mm = force_series_span / (REFERENCE_STROKE_MM - fixed_other_span_mm)
    total_grid_points = 0
    for rate in RATES_N_PER_MM:
        stiffness = rate * 1000
        low = q_formula(5.0, 2.0, 1.0, stiffness)
        high = q_formula(50.0, 0.1, 0.0, stiffness)
        grid = dense_grid_coordinate_extrema(stiffness)
        total_grid_points += grid["points"]
        expected_span_mm = 1000 * (high - low)
        assert math.isclose(grid["nut_position_min_mm"], 1000 * low, abs_tol=1e-10)
        assert math.isclose(grid["nut_position_max_mm"], 1000 * high, abs_tol=1e-10)
        assert math.isclose(grid["nut_position_span_mm"], expected_span_mm, abs_tol=1e-10)
        assert math.isclose(expected_span_mm, fixed_other_span_mm + force_series_span / rate, abs_tol=1e-10)
        # A constant preload adds a constant displacement and cannot shrink range.
        preload = 30.0
        shifted_low = low - preload / stiffness
        shifted_high = high - preload / stiffness
        assert math.isclose(shifted_high - shifted_low, high - low, abs_tol=1e-12)
        rows.append({
            "spring_rate_N_per_mm": rate,
            "spring_rate_N_per_m": stiffness,
            "spring_force_change_for_0_1mm_lag_N": rate * LAG_MM,
            "nut_to_B_spring_deflection_min_mm_with_zero_preload": force_series_min / rate,
            "nut_to_B_spring_deflection_max_mm_with_zero_preload": force_series_max / rate,
            "nut_to_B_spring_deflection_span_mm": force_series_span / rate,
            "nut_position_min_mm_with_zero_preload": 1000 * low,
            "nut_position_max_mm_with_zero_preload": 1000 * high,
            "nut_position_span_mm": expected_span_mm,
            "fits_static_25mm_reference_envelope_with_adjustable_zero": expected_span_mm <= REFERENCE_STROKE_MM,
            "dense_grid_coordinate_check": grid,
        })
    assert all(x["nut_position_span_mm"] > y["nut_position_span_mm"] for x, y in zip(rows, rows[1:]))
    assert all(x["spring_force_change_for_0_1mm_lag_N"] < y["spring_force_change_for_0_1mm_lag_N"] for x, y in zip(rows, rows[1:]))
    assert math.isclose(q_formula(50.0, 2.0, 0.0, math.inf) - q_formula(5.0, 2.0, 1.0, math.inf), fixed_other_span_mm / 1000, abs_tol=1e-12)
    data = {
        "status": "STATIC_ARITHMETIC_CHECKS_PASSED_NOT_FORCE_QUALIFICATION",
        "experiment": "Leadscrew plus actuator-internal series spring; static comparison correction",
        "units": "SI in formulas; displayed stiffness N/mm and lengths mm",
        "parameters": {
            "g_m_per_s2": G, "A_kg": A, "B_kg": B, "S_kg_range": list(S_RANGE),
            "force_N_range": list(FORCE_RANGE), "gravity_angle_degrees_range": [0.0, 90.0],
            "upper_R_A_stiffness_N_per_m": K_UPPER, "upper_external_damping_Ns_per_m": 0.0,
            "upper_spring_travel": "ideal unlimited, prescribed stiffness retained",
            "B_S_cell_stiffness_N_per_m": K_CELL,
            "B_S_external_connection": "rigid with finite load-cell stiffness",
            "actuator_series_spring_external_damping_Ns_per_m": 0.0,
            "reference_nut_stroke_mm": REFERENCE_STROKE_MM,
            "relative_nut_to_B_lag_example_mm": LAG_MM,
        },
        "equations": {
            "gravity_axis": "g_axis = g*cos(theta); theta=0 aligns gravity toward contact; theta=90 gives zero axis gravity",
            "load_cell_static_force": "F_cell = F - S*g_axis",
            "actuator_series_spring_static_force": "F_s = F - (B+S)*g_axis",
            "upper_static_force": "F_upper = F - (A+B+S)*g_axis",
            "series_spring_force": "F_s = preload + k_s*(x_nut - x_B)",
            "nut_relative_A_position_with_zero_preload": "q = [F-(A+B+S)*g*cos(theta)]/k_upper + [F-(B+S)*g*cos(theta)]/k_s + [F-S*g*cos(theta)]/k_cell",
            "position_with_fixed_preload": "q(preload) = q(0) - preload/k_s; fixed preload shifts zero, not span",
            "relative_lag_force_sensitivity": "Delta F_s = k_s*Delta(x_nut-x_B), NOT closed-loop contact-force error",
            "reflected_inertia_topology": "ell=lead/(2*pi); J/ell^2 belongs to nut coordinate, not automatically to B or S; nut and tool motion become separate states",
            "total_fixed_robot_position_span_mm": "other_span_mm + [50-(5-(B+S_max)*g)]/rate_N_per_mm",
        },
        "analytical_extrema": {
            "min_q_at": {"F_N": 5.0, "S_kg": 2.0, "theta_deg": 0.0},
            "max_q_at": {"F_N": 50.0, "S_kg": "any value in range", "theta_deg": 90.0},
            "argument": "q increases with F and decreases with positive gravity cosine and downstream mass; dense coordinate grid independently confirms endpoint extrema",
            "actuator_spring_force_min_N": force_series_min,
            "actuator_spring_force_max_N": force_series_max,
            "actuator_spring_force_span_N": force_series_span,
            "upper_and_cell_position_span_mm": fixed_other_span_mm,
            "minimum_rate_for_25mm_static_reference_N_per_mm": threshold_n_per_mm,
        },
        "scope_and_limits": [
            "Quasi-static normal contact with flange axis aligned to surface normal; theta changes gravity projection, not sanding drag.",
            "The added spring is inside the actuator, between nut and B; it does not replace the upper spring or the finite B-S cell.",
            "Spring is an ideal bidirectional linear element. Negative force endpoints require a suitable tension/compression or opposed/preloaded implementation; no part feasibility is established.",
            "Robot and tool positions are held fixed for the envelope; recentering, a longer screw, or manually reset preload changes the required position envelope.",
            "25 mm is a comparison reference, not a universal limit on leadscrew travel. Static span fitting it leaves unspecified dynamic reserve.",
            "Nut inertia and motor friction/backlash are not assigned to B; their separate state and coupling must be modeled in a future dynamic candidate.",
            "No bump waveform, tool inertia, finite-cell vibration, control timing, motor torque/current/voltage, spring packaging, friction or contact stop is simulated here.",
            "Tool inertia and finite-cell modes remain relevant in a dynamic spring-coupled design. Softness alone does not establish +/-0.5 N contact performance.",
            "Existing direct-drive screw recordings remain historical reference evidence; these calculations do not rerank actuator controllers by force qualification.",
        ],
        "checks": {
            "independent_dense_grid_per_rate": {"force_points": 91, "mass_points": 39, "angle_points": 181},
            "total_streamed_grid_points": total_grid_points,
            "extrema_absolute_tolerance_mm": 1e-10,
            "static_force_balance_checked_at_each_grid_point": True,
            "constant_preload_span_invariance_checked": True,
            "sensitivity_and_span_monotonicity_checked": True,
            "infinite_stiffness_static_limit_checked": True,
            "stored_arrays_or_native_histories": 0,
            "simulations_or_training_runs": 0,
        },
        "rows": rows,
        "source": {"script_relative_path": str(Path(__file__).resolve().relative_to(ROOT)), "sha256": sha256(Path(__file__))},
    }
    write_json(VALIDATION / "static_tradeoff.json", data)
    FIGURE.parent.mkdir(parents=True, exist_ok=True)
    FIGURE.write_text(figure(rows, threshold_n_per_mm))
    footprint_paths = [p for p in EXPERIMENT.rglob("*") if p.is_file()] + [p for p in VALIDATION.rglob("*") if p.is_file()] + [FIGURE]
    footprint = sum(p.stat().st_size for p in footprint_paths)
    assert footprint <= RETAINED_FORECAST_BYTES
    print(json.dumps({
        "status": data["status"], "grid_points": total_grid_points,
        "current_shared_array_bytes": forecast["current_unique_mat_npz_bytes"],
        "new_array_bytes": 0, "retained_bytes_before_final_manifest": footprint,
        "minimum_rate_for_25mm_reference_N_per_mm": threshold_n_per_mm,
        "rows": [{key: row[key] for key in ["spring_rate_N_per_mm", "spring_force_change_for_0_1mm_lag_N", "nut_position_span_mm"]} for row in rows],
        "figure": str(FIGURE.relative_to(ROOT)),
    }, indent=2))


if __name__ == "__main__":
    main()
