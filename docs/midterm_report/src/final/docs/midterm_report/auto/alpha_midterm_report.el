(TeX-add-style-hook "alpha_midterm_report"
 (lambda ()
    (LaTeX-add-bibliographies
     "bibliography")
    (TeX-run-style-hooks
     "longtable"
     "placeins"
     "section"
     "todonotes"
     "colorinlistoftodos"
     "obeyDraft"
     "geometry"
     "ifdraft"
     "amssymb"
     "graphicx"
     "ctable"
     "hyperref"
     "amsfonts"
     "amsmath"
     "inputenc"
     "utf8"
     "latex2e"
     "rep10"
     "report"
     "10pt"
     "letterpaper"
     "final"
     "overview"
     "software_architecture"
     "node_details"
     "demo_code")))

