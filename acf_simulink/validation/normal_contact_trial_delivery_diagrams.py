"""Package the reviewed native Simulink PNG pages without changing the model.

Run with: uv run --with reportlab python <this file>
The PDF retains each PNG at its full resolution on an appropriately sized page.
It is a raster export, with PDF bookmarks for native subsystem names.
"""

import hashlib
import json
import shutil
import sys
from pathlib import Path

from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    work = Path.home() / ".local/state/actuator-model/inertial-delivery-private-v8-layoutd-20260930"
    round_name = sys.argv[1] if len(sys.argv) > 1 else "review_clean"
    source = work / "figures" / round_name / "pages.json"
    pages = json.loads(source.read_text())["pages"]
    assert len(pages) == 13
    target = root / "generated/active_flange_native_model_pages_20260930.pdf"
    document = canvas.Canvas(str(target), pageCompression=1)
    document.setTitle("Active contact flange: native Simulink block diagrams")
    document.setAuthor("Active contact flange model")
    document.setSubject("Thirteen reviewed native block pages, exported as full-resolution raster images")
    exported = root / "generated/native_block_pages_20260930"
    exported.mkdir(exist_ok=True)
    manifest = {"artifact": str(target.relative_to(root)), "format": "PDF with lossless raster PNG pages", "modelSHA256": sha256(work / "generated/acf_force_control_simple.slx"), "pages": []}
    for index, page in enumerate(pages, 1):
        image = Path(page["image"])
        public_image = exported / image.name
        shutil.copy2(image, public_image)
        reader = ImageReader(str(image))
        pixels_x, pixels_y = reader.getSize()
        width, height = pixels_x / 2, pixels_y / 2
        document.setPageSize((width + 36, height + 64))
        bookmark = f"page_{index:02d}"
        document.bookmarkPage(bookmark)
        document.addOutlineEntry(page["path"], bookmark, level=0)
        document.setFont("Helvetica", 12)
        document.drawString(18, height + 40, f"{index:02d}. {page['path']}")
        document.drawImage(reader, 18, 20, width=width, height=height, mask="auto")
        document.showPage()
        manifest["pages"].append({"page": index, "nativePath": page["path"], "image": str(public_image.relative_to(root)), "imageSHA256": sha256(image), "imagePixels": [pixels_x, pixels_y]})
    document.save()
    manifest["PDFSHA256"] = sha256(target)
    manifest["sourcePageManifestSHA256"] = sha256(source)
    output = root / "validation/normal_contact_trial_delivery_diagrams_20260930.json"
    output.write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"PDF": str(target), "pageCount": len(pages), "bytes": target.stat().st_size, "SHA256": manifest["PDFSHA256"]}))


if __name__ == "__main__":
    main()
