"""Draw the user-prescribed mechanical and measurement layout as native SVG."""
from pathlib import Path
from html import escape

ROOT = Path(__file__).resolve().parents[3]
FIGURE = ROOT / 'reports/figures/fixed-upper-rigid-lower-20261005.svg'


def main():
    items = []
    def text(x, y, message, size=15, fill='#24374b', weight='400', anchor='start'):
        items.append(f'<text x="{x}" y="{y}" font-size="{size}" fill="{fill}" font-weight="{weight}" text-anchor="{anchor}">{escape(message)}</text>')
    def box(x, y, w, h, fill, stroke, label, detail):
        items.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="{fill}" stroke="{stroke}" stroke-width="2"/>')
        text(x+w/2,y+31,label,18,weight='650',anchor='middle')
        for i,line in enumerate(detail):
            text(x+w/2,y+57+i*20,line,13,anchor='middle')
    def arrow(x1,y1,x2,y2,color='#50677a'):
        items.append(f'<path d="M{x1} {y1} L{x2} {y2}" fill="none" stroke="{color}" stroke-width="2.3" marker-end="url(#arrow)"/>')
    text(42,44,'Fixed mechanics for the current prototype',26,weight='700')
    text(42,75,'Control and sensing must work around these prescribed connections.',16,fill='#526578')
    box(42,116,105,110,'#edf2f7','#8297ab','Robot R',['Issued motion'])
    box(180,116,182,110,'#ddf3ed','#328b78','Upper spring',['20 N/mm; damping = 0','Ideal unlimited travel'])
    box(394,116,105,110,'#edf2f7','#8297ab','Body A',['0.4 / 1.5 kg'])
    box(532,116,156,110,'#e4eefb','#4875b4','Voice coil',['Internal guide friction','25 mm actuator stroke'])
    box(721,116,105,110,'#edf2f7','#8297ab','Holder B',['0.6 kg'])
    box(859,116,178,110,'#f4e8f5','#a36aab','Rigid connection',['Multi-axis load cell','Finite stiffness retained'])
    box(1070,116,125,110,'#fff1d4','#ad8c3c','Tool S',['0.1–2 kg'])
    for x1,x2 in [(147,180),(362,394),(499,532),(688,721),(826,859),(1037,1070)]:
        arrow(x1,170,x2,170)
    items.append('<path d="M446 230 L446 277 L1132 277 L1132 230" fill="none" stroke="#4875b4" stroke-width="2"/>')
    text(790,301,'Measure tool position S relative to body A',15,fill='#4875b4',anchor='middle')
    items.append('<rect x="42" y="327" width="1153" height="138" rx="14" fill="#f6f8fb" stroke="#d5dfe8"/>')
    text(65,356,'Workpiece-normal force measurement',18,weight='650')
    box(65,374,220,67,'#ffffff','#a36aab','B–S cell vector',[])
    box(352,374,295,67,'#ffffff','#328b78','Add tool inertia',[])
    box(714,374,292,67,'#ffffff','#4875b4','Project onto normal',[])
    arrow(285,407,352,407);arrow(647,407,714,407);arrow(1006,407,1170,407)
    text(1107,396,'Pressing force',15,weight='600',anchor='middle')
    text(42,498,'An accelerometer on S supplies dynamic inertia; force, acceleration and normal estimates share one timestamp.',14,fill='#526578')
    text(42,524,'Upper spring travel is unconstrained. Contact loss and the voice coil’s travel stops still fail a run.',14,fill='#526578')
    svg = '<svg xmlns="http://www.w3.org/2000/svg" width="1240" height="555" viewBox="0 0 1240 555" role="img" aria-labelledby="title desc">'
    svg += '<title id="title">Fixed upper spring and rigid lower connection</title><desc id="desc">Robot R, a prescribed 20 newton per millimetre undamped upper spring, body A, voice coil, holder B, finite-stiffness multi-axis load cell and tool S. Position measures S relative to A. Workpiece-normal force is reconstructed from load and tool inertia.</desc>'
    svg += '<defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto-start-reverse"><path d="M0 0 L8 4 L0 8 Z" fill="#50677a"/></marker></defs><rect width="1240" height="555" fill="#ffffff"/><g font-family="Inter,DejaVu Sans,Arial,sans-serif">'
    svg += ''.join(items) + '</g></svg>\n'
    FIGURE.parent.mkdir(parents=True,exist_ok=True)
    FIGURE.write_text(svg)
    print(FIGURE)


if __name__ == '__main__':
    main()
