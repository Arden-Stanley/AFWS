;; -*- lexical-binding: t; -*-

(TeX-add-style-hook
 "IEEE-conference-template-062824"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-class-options
                     '(("IEEEtran" "conference")))
   (TeX-add-to-alist 'LaTeX-provided-package-options
                     '(("cite" "") ("amsmath" "") ("amssymb" "") ("amsfonts" "") ("algorithmic" "") ("graphicx" "") ("textcomp" "") ("xcolor" "")))
   (TeX-run-style-hooks
    "latex2e"
    "IEEEtran"
    "IEEEtran10"
    "cite"
    "amsmath"
    "amssymb"
    "amsfonts"
    "algorithmic"
    "graphicx"
    "textcomp"
    "xcolor")
   (TeX-add-symbols
    "BibTeX")
   (LaTeX-add-labels
    "fig:CAD_Model"
    "fig:Wiring_Diagram"
    "fig:CNN1"
    "fig:Lidar_Diagram"
    "fig:StateMachine"
    "fig:CNN2"
    "fig:CNN3"
    "fig:CNN4"
    "fig:CNN5"
    "fig:CNN6")
   (LaTeX-add-bibitems
    "b1"
    "b2"
    "b3"
    "b4"
    "b5"
    "b6"
    "b7"))
 :latex)

