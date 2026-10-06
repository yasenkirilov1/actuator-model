"""Verify unchanged saved native block/wire contracts outside current FF."""
from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from zipfile import ZipFile


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical(data: bytes) -> object:
    root = ET.fromstring(data)
    names = {b.attrib["SID"]: b.attrib["Name"] for b in root.findall("Block")}

    def visit(element: ET.Element) -> object:
        attrs = {k: v for k, v in element.attrib.items() if k != "SID"}
        value = element.text.strip() if element.text else ""
        if element.tag == "P" and attrs.get("Name") == "SIDHighWatermark":
            value = "<ignored-document-id>"
        if element.tag == "P" and attrs.get("Name") in {"Src", "Dst"}:
            sid, port = value.split("#", 1)
            value = names[sid] + "#" + port
        return element.tag, attrs, value, [visit(child) for child in element]

    return visit(root)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline")
    parser.add_argument("candidate")
    parser.add_argument("output")
    args = parser.parse_args()
    baseline, candidate, output = map(Path, [args.baseline, args.candidate, args.output])
    assert not output.exists()
    ff_system = "simulink/systems/system_1324.xml"
    with ZipFile(baseline / "acf_rl_native.slx") as old, ZipFile(candidate / "acf_rl_native.slx") as new:
        systems = sorted(n for n in old.namelist() if n.startswith("simulink/systems/") and n.endswith(".xml"))
        assert systems == sorted(n for n in new.namelist() if n.startswith("simulink/systems/") and n.endswith(".xml"))
        changed = [n for n in systems if canonical(old.read(n)) != canonical(new.read(n))]
        assert changed == [ff_system], changed
        before = ET.fromstring(old.read(ff_system))
        after = ET.fromstring(new.read(ff_system))
        prior_names = {b.attrib["Name"] for b in before.findall("Block")}
        after_names = {b.attrib["Name"] for b in after.findall("Block")}
        assert after_names - prior_names == {"Present-time acceleration lead for current FF"}
        assert prior_names - after_names == set()
    metadata = json.loads((candidate / "metadata.json").read_text())
    assert metadata["privateModelSha256"] == sha(candidate / "acf_rl_native.slx")
    contract = metadata["accelerationLeadContract"]
    assert contract["measurementFrameUnchanged"] and contract["causal"]
    report = {
        "status": "saved_graph_current_feedforward_only_verified",
        "physicalEpisodes": 0,
        "baselineModelSha256": sha(baseline / "acf_rl_native.slx"),
        "candidateModelSha256": metadata["privateModelSha256"],
        "sourcePhysicalModelSha256": metadata["sourceModelSha256"],
        "nativeSystemCount": len(systems),
        "unchangedNativeSystems": len(systems) - 1,
        "changedNativeSystems": changed,
        "newNativeBlocks": ["Present-time acceleration lead for current FF"],
        "normalization": "Ignore document-only SID renumbering; map every wire SID to its actual block name. Preserve all block parameters, other system attributes and wire structure.",
        "conclusion": "Only acceleration current-feedforward subsystem changed. Physical plant, sensor/ADC processing, contact-force reconstruction, motion observer, normal projection, robot timing/gate and other controller blocks match the original finite-cell graph.",
        "leadContract": contract,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "leadContract"}, indent=2))


if __name__ == "__main__":
    main()
