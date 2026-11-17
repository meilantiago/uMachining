#!/usr/bin/env python3
"""Convierte un informe Markdown del repo a PDF usando Microsoft Edge sin ventana.

Soporta lo que usan los informes: títulos (#, ##, ###), párrafos, **negrita**,
*itálica*, `código`, listas con - o 1., bloques de código, tablas, citas (>) e
imágenes ![leyenda](archivo.png). Un título de nivel 1 que empieza con "Parte"
abre página nueva.

Uso:
    python tools/informe_pdf.py simulaciones/prueba2/RESULTADOS.md [salida.pdf]
"""
from __future__ import annotations

import html
import pathlib
import re
import subprocess
import sys
import tempfile

EDGE = [
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
]

CSS = """
@page { size: A4; margin: 18mm 16mm 18mm 16mm; }
body { font-family: "Segoe UI", Calibri, Arial, sans-serif; font-size: 10.2pt; line-height: 1.45;
       color: #1b1b1a; }
h1.titulo { font-size: 21pt; margin: 0 0 6pt 0; color: #0b0b0b; }
h1.parte { page-break-before: always; font-size: 16pt; color: #fff; background: #2a5f9e;
           padding: 7pt 10pt; border-radius: 4px; margin: 0 0 12pt 0; }
h1 { font-size: 16pt; }
h2 { font-size: 13pt; color: #1c4f8a; border-bottom: 1px solid #c9d6e6; padding-bottom: 2pt;
     margin: 16pt 0 6pt 0; page-break-after: avoid; }
h3 { font-size: 11pt; color: #1b1b1a; margin: 12pt 0 4pt 0; page-break-after: avoid; }
p { margin: 4pt 0 6pt 0; }
ul, ol { margin: 3pt 0 6pt 0; padding-left: 18pt; }
li { margin: 2pt 0; }
code { font-family: Consolas, monospace; font-size: 9pt; background: #f1f0ec; padding: 0 2pt;
       border-radius: 2px; }
pre { font-family: Consolas, monospace; font-size: 9pt; background: #f4f3ef; border-left: 3px solid #2a78d6;
      padding: 6pt 8pt; margin: 6pt 0 8pt 0; white-space: pre-wrap; page-break-inside: avoid; }
pre code { background: none; padding: 0; }
table { border-collapse: collapse; margin: 6pt 0 10pt 0; font-size: 8.8pt; width: auto;
        page-break-inside: avoid; }
th, td { border: 1px solid #cfcdc6; padding: 3pt 6pt; vertical-align: top; text-align: left; }
th { background: #eef2f7; }
tr:nth-child(even) td { background: #fafaf8; }
blockquote { margin: 6pt 0; padding: 6pt 10pt; background: #f4f7fb; border-left: 3px solid #2a5f9e;
             font-style: italic; }
figure { margin: 8pt 0 12pt 0; page-break-inside: avoid; text-align: center; }
figure img { max-width: 100%; max-height: 120mm; }
figcaption { font-size: 8.8pt; color: #52514e; margin-top: 2pt; }
.indice { color: #52514e; }
"""


def inline(t: str) -> str:
    t = html.escape(t, quote=False)
    t = re.sub(r"`([^`]+)`", r"<code>\1</code>", t)
    t = re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", t)
    t = re.sub(r"(?<![\w*])\*(?!\s)(.+?)(?<!\s)\*(?![\w*])", r"<em>\1</em>", t)
    return t


def celdas(linea: str) -> list[str]:
    linea = linea.strip().strip("|")
    partes = re.split(r"(?<!\\)\|", linea)
    return [p.strip().replace("\\|", "|") for p in partes]


def convertir(md: str, base: pathlib.Path) -> str:
    lineas = md.replace("\r\n", "\n").split("\n")
    out: list[str] = []
    i = 0
    primer_h1 = True
    while i < len(lineas):
        ln = lineas[i]
        s = ln.strip()
        if not s:
            i += 1
            continue
        if s.startswith("```"):
            i += 1
            bloque = []
            while i < len(lineas) and not lineas[i].strip().startswith("```"):
                bloque.append(lineas[i])
                i += 1
            i += 1
            out.append("<pre><code>" + html.escape("\n".join(bloque)) + "</code></pre>")
            continue
        m = re.match(r"^(#{1,3})\s+(.*)$", s)
        if m:
            nivel, texto = len(m.group(1)), m.group(2)
            if nivel == 1 and primer_h1:
                out.append(f'<h1 class="titulo">{inline(texto)}</h1>')
                primer_h1 = False
            elif nivel == 1 and texto.startswith("Parte"):
                out.append(f'<h1 class="parte">{inline(texto)}</h1>')
            else:
                out.append(f"<h{nivel}>{inline(texto)}</h{nivel}>")
            i += 1
            continue
        m = re.match(r"^!\[(.*?)\]\((.*?)\)$", s)
        if m:
            src = (base / m.group(2)).resolve().as_uri()
            cap = f"<figcaption>{inline(m.group(1))}</figcaption>" if m.group(1) else ""
            out.append(f'<figure><img src="{src}">{cap}</figure>')
            i += 1
            continue
        if s.startswith("|"):
            filas = []
            while i < len(lineas) and lineas[i].strip().startswith("|"):
                filas.append(lineas[i])
                i += 1
            cab = celdas(filas[0])
            cuerpo = [celdas(f) for f in filas[2:]]
            h = "<table><thead><tr>" + "".join(f"<th>{inline(c)}</th>" for c in cab) + "</tr></thead><tbody>"
            for f in cuerpo:
                h += "<tr>" + "".join(f"<td>{inline(c)}</td>" for c in f) + "</tr>"
            out.append(h + "</tbody></table>")
            continue
        if s.startswith(">"):
            bloque = []
            while i < len(lineas) and lineas[i].strip().startswith(">"):
                bloque.append(lineas[i].strip()[1:].strip())
                i += 1
            out.append("<blockquote>" + inline(" ".join(bloque)) + "</blockquote>")
            continue
        m = re.match(r"^(-|\d+\.)\s+(.*)$", s)
        if m and not ln.startswith(" "):
            ordenada = m.group(1) != "-"
            inicio = int(m.group(1)[:-1]) if ordenada else 1
            items: list[list[str]] = []
            sub: list[list[str]] = []
            while i < len(lineas):
                l2 = lineas[i]
                m2 = re.match(r"^(-|\d+\.)\s+(.*)$", l2)
                if m2 and not l2.startswith(" ") and ((m2.group(1) != "-") == ordenada):
                    items.append([m2.group(2)])
                    sub.append([])
                    i += 1
                elif l2.startswith("  ") and l2.strip() and items:
                    m3 = re.match(r"^\s+-\s+(.*)$", l2)
                    if m3:
                        sub[-1].append(m3.group(1))
                    else:
                        items[-1].append(l2.strip())
                    i += 1
                elif not l2.strip() and i + 1 < len(lineas) and lineas[i + 1].startswith("  ") and items:
                    i += 1
                else:
                    break
            tag = "ol" if ordenada else "ul"
            attr = f' start="{inicio}"' if ordenada and inicio != 1 else ""
            h = f"<{tag}{attr}>"
            for it, sb in zip(items, sub):
                h += "<li>" + "<br>".join(inline(x) for x in it)
                if sb:
                    h += "<ul>" + "".join(f"<li>{inline(x)}</li>" for x in sb) + "</ul>"
                h += "</li>"
            out.append(h + f"</{tag}>")
            continue
        parrafo = []
        while i < len(lineas):
            l2 = lineas[i].strip()
            if (not l2 or l2.startswith(("#", "|", "```", ">", "![")) or re.match(r"^(-|\d+\.)\s", l2)):
                break
            parrafo.append(l2)
            i += 1
        if not parrafo:          # línea que ningún caso consumió (p. ej. "   - x" fuera de una lista)
            parrafo.append(re.sub(r"^(-|\d+\.)\s+", "", lineas[i].strip()))
            i += 1
        out.append("<p>" + "<br>".join(inline(x) for x in parrafo) + "</p>")
    return "\n".join(out)


def main():
    md_path = pathlib.Path(sys.argv[1]).resolve()
    pdf_path = pathlib.Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else md_path.with_suffix(".pdf")
    md = md_path.read_text(encoding="utf-8-sig")
    titulo = re.search(r"^#\s+(.*)$", md, re.M).group(1)
    cuerpo = convertir(md, md_path.parent)
    doc = (f'<!doctype html><html lang="es"><head><meta charset="utf-8"><title>{html.escape(titulo)}</title>'
           f"<style>{CSS}</style></head><body>{cuerpo}</body></html>")
    edge = next((e for e in EDGE if pathlib.Path(e).exists()), None)
    if edge is None:
        sys.exit("No se encontró Microsoft Edge.")
    with tempfile.TemporaryDirectory() as tmp:
        h = pathlib.Path(tmp) / "informe.html"
        h.write_text(doc, encoding="utf-8")
        subprocess.run([edge, "--headless", "--disable-gpu", "--no-pdf-header-footer",
                        f"--user-data-dir={tmp}\\perfil", f"--print-to-pdf={pdf_path}", h.as_uri()],
                       check=True, capture_output=True, timeout=180)
    print(pdf_path)


if __name__ == "__main__":
    main()
