"""Verify that the native layout cleanup changes only unbound line graphics."""

import hashlib
import json
import sys
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path


def signature(node):
    text = node.text or ""
    if len(node) and not text.strip():
        text = ""
    return node.tag, sorted(node.attrib.items()), text, [signature(c) for c in node]


def read_model(filename):
    blocks, edges, parameters, dangling = {}, [], {}, []
    with zipfile.ZipFile(filename) as archive:
        workspace = hashlib.sha256(archive.read("simulink/modelWorkspace.mxarray")).hexdigest()
        for name in archive.namelist():
            if not name.startswith("simulink/systems/") or not name.endswith(".xml"):
                continue
            system = ET.fromstring(archive.read(name))
            parameters[name] = [signature(p) for p in system.findall("P")]
            for block in system.iter("Block"):
                blocks[name, block.attrib["SID"]] = signature(block)
            for line in system.iter("Line"):
                props = {p.attrib["Name"]: p.text for p in line.findall("P")}
                source = props.get("Src")
                for leaf in [line, *line.iter("Branch")]:
                    values = {p.attrib["Name"]: p.text for p in leaf.findall("P")}
                    destination = values.get("Dst")
                    if source and destination:
                        edges.append((name, source, destination))
                    if not leaf.findall("Branch") and not destination:
                        dangling.append((name, source, values.get("Points")))
    return blocks, sorted(edges), parameters, dangling, workspace


def main():
    before_file, after_file, output_file = map(Path, sys.argv[1:])
    before = read_model(before_file)
    after = read_model(after_file)
    assert before[0] == after[0], "Native block parameters or positions changed"
    assert before[1] == after[1], "A connected source/destination edge changed"
    assert before[2] == after[2], "Native system parameters changed"
    assert before[4] == after[4], "Model Workspace changed during line cleanup"
    assert len(before[3]) == 33 and not after[3], "Expected33 unbound leaves and no remaining stub"
    report = {
        "status": "complete",
        "newNativeSimulations": 0,
        "beforeModelSHA256": hashlib.sha256(before_file.read_bytes()).hexdigest(),
        "modelSHA256": hashlib.sha256(after_file.read_bytes()).hexdigest(),
        "nativeBlockParametersIdentical": True,
        "nativeSystemParametersIdentical": True,
        "connectedSourceDestinationEdgesIdentical": True,
        "modelWorkspaceIdentical": True,
        "blockCount": len(after[0]),
        "systemCount": len(after[2]),
        "connectedEdgeCount": len(after[1]),
        "destinationlessLeavesBefore": len(before[3]),
        "destinationlessLeavesAfter": len(after[3]),
        "removedDestinationlessLeaves": before[3],
    }
    output_file.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "removedDestinationlessLeaves"}))


if __name__ == "__main__":
    main()
