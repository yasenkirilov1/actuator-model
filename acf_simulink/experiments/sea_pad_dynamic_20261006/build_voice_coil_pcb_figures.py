"""Versioned proposal diagrams; preserve all previously published study figures.

No simulation or quantitative replot runs here. Architecture is a design
proposal, separately labeled from measured or simulated evidence.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
RECIPE = Path(__file__).resolve().parent
OUTPUT = ROOT / "reports/figures"
AUDIT = ROOT / "acf_simulink/validation/sea_pad_dynamic_20261006/report"
NS = "http://www.w3.org/2000/svg"
ET.register_namespace("", NS)


def bound(path):
    return {"path": str(path.relative_to(ROOT)),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def build_mechanics(proposal):
    original = OUTPUT / "prototype-mechanics-pad5mm-20261006.svg"
    svg = ET.parse(original).getroot()
    replacements = {
        "Prescribed mechanics with the spring-driven screw": "Voice-coil prototype and integrated sensing on S",
        "Screw + spring": "Voice coil",
        "Independent nut + encoder": "Direct current-to-force",
        "Internal spring: 5 N/mm": "25 mm actuator stroke",
        "Accelerometer on S": "Sensing PCB on S",
    }
    for text in svg.iter(f"{{{NS}}}text"):
        if text.text in replacements:
            text.text = replacements[text.text]
    svg.find(f"{{{NS}}}title").text = "Voice-coil prototype, PCB on S and axial force reconstruction"
    svg.find(f"{{{NS}}}desc").text = (
        "Robot R, prescribed undamped 20 N/mm upper spring, body A, voice coil, "
        "holder B, finite-stiffness one-axis cell, and tool S with an integrated "
        "sensing PCB and 77 mm by 5 mm pad. Include PCB mass in S. Tared cell "
        "force plus S inertia reconstruct axial force; drag separation is deferred.")
    path = OUTPUT / "prototype-mechanics-voice-coil-pcb-20261006.svg"
    ET.ElementTree(svg).write(path, encoding="unicode")
    return {"figure": bound(path), "sources": [bound(original), bound(proposal)],
            "scope": "Proposed voice-coil mechanics and sensing locations; no new simulation."}


def build_pcb(proposal):
    svg = ET.Element(f"{{{NS}}}svg", {
        "width": "1320", "height": "760", "viewBox": "0 0 1320 760",
        "role": "img", "aria-labelledby": "title desc"})
    def element(tag, attrs, parent=svg, text=None):
        node = ET.SubElement(parent, f"{{{NS}}}{tag}", {k: str(v) for k, v in attrs.items()})
        node.text = text
        return node
    element("title", {"id": "title"}, text="Proposed integrated sensing and control PCB")
    element("desc", {"id": "desc"}, text=(
        "On S, amplify the B-S axial load-cell signal and sample it together "
        "with three ADXL354C analog acceleration outputs using ADS131M04. "
        "The microcontroller runs a local 10 kHz force loop and receives S-A "
        "position. Programming, calibration and robot communication use the "
        "same board. An external bipolar coil driver on A receives the command. "
        "Architecture proposed, not hardware-qualified."))
    defs = element("defs", {})
    marker = element("marker", {"id": "arrow", "markerWidth": 8, "markerHeight": 8,
        "refX": 7, "refY": 4, "orient": "auto-start-reverse"}, defs)
    element("path", {"d": "M0 0 L8 4 L0 8 Z", "fill": "#50677a"}, marker)
    element("rect", {"width": 1320, "height": 760, "fill": "#ffffff"})
    group = element("g", {"font-family": "DejaVu Sans,Arial,sans-serif"})
    def text(x, y, value, size=16, weight=400, anchor="start", color="#24374b"):
        element("text", {"x": x, "y": y, "font-size": size, "font-weight": weight,
            "text-anchor": anchor, "fill": color}, group, value)
    def box(x, y, w, h, title, lines, color="#4875b4", fill="#ffffff"):
        element("rect", {"x": x, "y": y, "width": w, "height": h, "rx": 12,
            "fill": fill, "stroke": color, "stroke-width": 2}, group)
        title_size = 16 if title == "Instrumentation amp" else 18
        text(x+w/2, y+30, title, title_size, 700, "middle")
        for i, line in enumerate(lines):
            text(x+w/2, y+58+24*i, line, 14, anchor="middle")
    def arrow(points, color="#50677a"):
        element("path", {"d": points, "fill": "none", "stroke": color,
            "stroke-width": 2.3, "marker-end": "url(#arrow)"}, group)
    text(35, 44, "One programmable board for sensing, calibration and force control", 25, 700)
    text(35, 77, "Proposed architecture · one-axis prototype · real noise, timing and robustness require assembled tests", 16, color="#526578")
    element("rect", {"x": 245, "y": 110, "width": 810, "height": 500, "rx": 18,
        "fill": "#eef8f5", "stroke": "#328b78", "stroke-width": 2}, group)
    text(267, 145, "CUSTOM SENSING / CONTROL PCB — RIGIDLY MOUNTED ON S", 18, 700, color="#24715f")

    box(25, 190, 180, 88, "B–S load cell", ["Axial bridge signal"], "#a36aab", "#f4e8f5")
    box(275, 185, 220, 105, "Instrumentation amp", ["Gain + analogue filter", "Excitation / reference"], "#a36aab")
    box(275, 365, 220, 110, "ADXL354C on S", ["Three analogue axes", "Matched conditioning"], "#328b78")
    box(535, 225, 210, 163, "ADS131M04", ["4 simultaneous inputs", "30 ksample/s/channel", "Common ADC clock", "SPI + data-ready"], "#4875b4")
    box(785, 208, 240, 164, "Programmable MCU", ["10 kHz force loop", "Force / inertia estimate", "Tool mass + gains", "Fault / enable logic"], "#4875b4")
    arrow("M205 234 L275 234")
    arrow("M495 237 L515 237 L515 266 L535 266")
    arrow("M495 421 L515 421 L515 347 L535 347")
    arrow("M745 292 L785 292")
    box(1090, 207, 205, 165, "Driver on A", ["Bipolar current drive", "Current limits + disable", "Coil + heat removal", "Power stage separate"], "#a16a29", "#fff6e9")
    arrow("M1025 292 L1090 292")
    box(25, 489, 180, 85, "S–A position", ["Separate sensor input"], "#4875b4", "#e4eefb")
    arrow("M205 531 L225 531 L225 580 L907 580 L907 548")
    box(775, 437, 250, 111, "Configuration / I/O", ["USB + debug + robot link", "Tare / presets / calibration", "Compact synchronized logs"], "#328b78")
    arrow("M902 437 L902 372")
    text(277, 523, "Bridge input + 3 acceleration outputs = 4 ADC channels", 15)
    text(277, 549, "Match installed filter phase; include PCB / mount mass in S", 15)
    text(35, 653, "Customizable", 18, 700)
    text(35, 680, "Gain / filter options", 15)
    text(350, 653, "Easy to use", 18, 700)
    text(350, 680, "One setup / tare interface", 15)
    text(680, 653, "Programmable", 18, 700)
    text(680, 680, "Local loop + saved calibration", 15)
    text(1000, 653, "Robust by design", 18, 700)
    text(1000, 680, "Protected wiring / fault disable", 15)
    text(35, 728, "ADC and accelerometer chips target < €100. Amplifier, MCU, PCB, connectors and driver are additional.", 15, color="#526578")
    path = OUTPUT / "integrated-sensing-pcb-20261006.svg"
    ET.indent(svg)
    ET.ElementTree(svg).write(path, encoding="unicode")
    return {"figure": bound(path), "sources": [bound(proposal)],
            "scope": "Proposed architecture and hardware targets, not measured PCB performance."}


if __name__ == "__main__":
    proposal = RECIPE / "voice_coil_pcb_proposal.json"
    config = json.loads(proposal.read_text())
    assert config["decision_basis"]["new_simulations"] == 0
    manifest = {"status": "versioned_prototype_proposal_diagrams",
                "builder": bound(Path(__file__)), "proposal": bound(proposal),
                "figures": [build_mechanics(proposal), build_pcb(proposal)],
                "no_scientific_results_changed": True}
    (AUDIT / "voice_coil_pcb_figure_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"figures": len(manifest["figures"]), "new_simulations": 0}))
