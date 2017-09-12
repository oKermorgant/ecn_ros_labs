\pdfminorversion=5
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{ecnreport}[09/09/2010, v1.0]
\LoadClass[12pt,a4paper]{article}

\usepackage[utf8]{inputenc}
\usepackage[namelimits,sumlimits]{amsmath}
\usepackage{amsfonts,amssymb}
\usepackage{graphicx}
\usepackage[hidelinks]{hyperref}
%\usepackage{hyperref}
\usepackage{fancyhdr}
% \usepackage{courier}
\usepackage{url}
\usepackage[hmargin=2cm,vmargin=3cm,headheight=1cm]{geometry}
\usepackage{array}
\usepackage{dirtree}
\usepackage{enumitem}
\usepackage{ok_abrv}
\usepackage{ecn_common}
\usepackage{titling}

\author{O. Kermorgant}


% Encadrements
\setlength{\fboxrule}{1.5pt}\setlength{\fboxsep}{2mm}

% De jolis en-têtes
\thispagestyle{fancyplain}
\pagestyle{fancyplain}

\newcommand{\stud}[1]
{
\lhead[]{\fancyplain{}{#1}}
}
\newcommand{\topic}[1]{
\rhead[]{\fancyplain{}{#1}}
}

\lfoot[]{\fancyplain{}{\theauthor}}
\rfoot[]{\raisebox{-.5\height}{\includegraphics[width=.15\linewidth]{LogoECNNew.pdf}}}

\newcolumntype{C}[1]{>{\centering\let\newline\\\arraybackslash\hspace{0pt}}m{#1}}

\newcommand{\inserttitle}[1]
{
\vspace{1cm}
\begin{center}
  \fbox{
\begin{tabular}{C{12cm}}
\Large \bf{#1}
\end{tabular}  }
\end{center}
}

\newcommand{\insertsubtitle}[1]
{
\begin{center}
\begin{tabular}{C{12cm}}
\large{\underline{ #1}}
\end{tabular}  
\end{center}
\medskip
}

% questions
\newlist{questions}{enumerate}{4}
\setlist[questions,1]{label={\medskip \bf{\underline{Q \arabic*}}}}
