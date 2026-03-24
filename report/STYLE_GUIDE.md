# ISS Report - Style Guide for Contributors

A comprehensive style guide to ensure consistency across all sections of the ISS Report.

---

## 📐 Document Formatting Standards

### Headings & Sections

```latex
% Section (Main heading)
\section{Section Title}

% Subsection (Sub heading)
\subsection{Subsection Title}

% Sub-subsection (if needed)
\subsubsection{Sub-subsection Title}
```

**Guidelines:**
- Use title case for all headings
- Keep headings concise and descriptive
- Maximum 3 levels of nesting
- Avoid excessive capitalization

### Paragraph Structure

```latex
\section{Main Topic}

Introduction paragraph explaining the section.

\subsection{Subtopic 1}

Content for subtopic 1. Start with a clear topic sentence.

Expand with details, supporting arguments, or explanations.

\subsection{Subtopic 2}

Content for subtopic 2...
```

**Guidelines:**
- Start each subsection with an introductory paragraph
- Use 1 blank line between paragraphs
- Keep paragraphs to 4-6 sentences
- Use consistent indentation

---

## 🔤 Text Formatting

### Font Styling

```latex
% Bold (strong emphasis)
\textbf{Important term}

% Italic (subtle emphasis)
\textit{Emphasized text}

% Underline
\underline{Underlined text}

% Typewriter (code/technical terms)
\texttt{code_example}

% Small caps
\textsc{Acronym}

% Combinations
\textbf{\textit{Bold italic}}
```

**Usage Guidelines:**
- **Bold:** For critical terms, definitions, key points
- *Italic:* For emphasis, references, foreign words
- `Typewriter:` For file names, code, technical identifiers, commands
- **Bold Italic:** Rare, use for strongly emphasized key concepts

### Lists

#### Unordered Lists
```latex
\begin{itemize}
    \item First point
    \item Second point
    \item Third point with
        \begin{itemize}
            \item Sub-point 1
            \item Sub-point 2
        \end{itemize}
\end{itemize}
```

#### Ordered Lists
```latex
\begin{enumerate}
    \item First step
    \item Second step
    \item Third step
\end{enumerate}
```

#### Description Lists
```latex
\begin{description}
    \item[Term 1] Definition or explanation
    \item[Term 2] Definition or explanation
\end{description}
```

**Guidelines:**
- Use bullet points for non-sequential items
- Use numbered lists for sequential processes
- Maximum 2 levels of nesting
- Keep items parallel in structure

---

## 📊 Tables

### Basic Table Structure

```latex
\begin{table}[h]
    \centering
    \caption{Descriptive table caption}
    \label{tab:unique-label}
    \begin{tabular}{|l|c|r|}
        \hline
        \textbf{Column 1} & \textbf{Column 2} & \textbf{Column 3} \\
        \hline
        Data 1 & Data 2 & Data 3 \\
        Data 4 & Data 5 & Data 6 \\
        \hline
    \end{tabular}
\end{table}
```

### Advanced Table (Professional)

```latex
\begin{table}[h]
    \centering
    \caption{Advanced table example with proper formatting}
    \label{tab:advanced}
    \begin{tabular}{lccr}
        \toprule
        \textbf{Parameter} & \textbf{Unit} & \textbf{Value} & \textbf{Range} \\
        \midrule
        Voltage & V & 12 & 10-15 \\
        Current & A & 2.5 & 1-5 \\
        Power & W & 30 & 20-50 \\
        \bottomrule
    \end{tabular}
\end{table}
```

**Table Guidelines:**
- Always include `\caption{}` and `\label{}`
- Use `[h]` placement (here), or `[htbp]` for flexibility
- Center tables with `\centering`
- Reference tables: "See Table~\ref{tab:unique-label}"
- Keep tables readable: max 5 columns
- Avoid vertical lines (use top/middle/bottom rules)
- Place tables before text that references them

---

## 📸 Figures & Images

### Image Insertion

```latex
\begin{figure}[h]
    \centering
    \includegraphics[width=0.8\textwidth]{images/diagram.png}
    \caption{Clear, descriptive caption}
    \label{fig:unique-label}
\end{figure}
```

### Image Sizing

```latex
% Full width
\includegraphics[width=\textwidth]{images/large-diagram.png}

% 80% width (recommended for most figures)
\includegraphics[width=0.8\textwidth]{images/diagram.png}

% Specific dimensions
\includegraphics[width=10cm, height=8cm]{images/diagram.png}

% By height only (maintains aspect ratio)
\includegraphics[height=5cm]{images/diagram.png}
```

**Figure Guidelines:**
- Store all images in `images/` folder
- Use descriptive file names: `architecture-diagram.png` (not `img1.png`)
- Supported formats: PNG, JPG, PDF
- Optimize image size before including (< 500KB per image)
- Always include descriptive captions
- Use `\label{}` and reference with `Figure~\ref{fig:label}`
- Place figures after first reference in text
- Use high-resolution images (300+ DPI for printing)

---

## 🔗 Cross-References & Citations

### Labels and References

```latex
% Create a label
\label{sec:methodology}

% Reference it
As discussed in Section~\ref{sec:methodology}...
See Table~\ref{tab:results} for details.
Figure~\ref{fig:architecture} shows the system design.

% Equation reference
Equation~\eqref{eq:energy-formula} demonstrates...
```

### Citation Formatting

```latex
% Single citation
According to Smith et al.~\cite{smith2023}, ...

% Multiple citations
Previous work~\cite{smith2023,jones2022} shows...

% Citation with page number
\cite[p. 45]{smith2023}

% As footnote
This was proven\footnote{\cite{smith2023}}.
```

**Reference Guidelines:**
- Use unique, descriptive labels: `sec:`, `fig:`, `tab:`, `eq:` prefixes
- Always cite sources in `references.bib`
- Maintain consistent citation style (IEEE style in this project)
- Reference figures/tables BEFORE they appear
- Use `~` (non-breaking space) before references: `Table~\ref{}`

---

## 🔢 Mathematics & Equations

### Inline Equations

```latex
The energy formula is $E = mc^2$.

For efficiency: $\eta = \frac{P_{out}}{P_{in}} \times 100\%$
```

### Display Equations

```latex
% Numbered equation
\begin{equation}
    \label{eq:efficiency}
    \eta = \frac{P_{out}}{P_{in}} \times 100\%
\end{equation}

Reference this as Equation~\eqref{eq:efficiency}.

% Multiple equations (aligned)
\begin{align}
    E &= mc^2 \label{eq:einstein} \\
    P &= VI \label{eq:power}
\end{align}
```

**Math Guidelines:**
- Inline math uses single `$...$`
- Display math uses `\[...\]` or `\begin{equation}...\end{equation}`
- Always label important equations
- Define variables before using them
- Use proper mathematical notation

---

## 📝 Common Elements

### Definitions & Important Terms

```latex
\textbf{Term:} Definition of the term in complete sentence.

\textit{Electronic Control Unit (ECU):} A microcomputer that controls 
various subsystems of the vehicle.
```

### Key Points & Important Information

```latex
\begin{itemize}
    \item[\textbf{Key Point 1:}] This is an important finding...
    \item[\textbf{Key Point 2:}] Another critical aspect...
\end{itemize}
```

### Callout Boxes (if available in preamble)

```latex
\begin{tcolorbox}
    \textbf{Important:} This critical information requires attention.
\end{tcolorbox}
```

---

## 🔤 Terminology & Abbreviations

### First Use of Abbreviations

```latex
% First occurrence: spell out with abbreviation
The Electronic Control Unit (ECU) manages all vehicle systems. 
Subsequent ECU references use the abbreviation.

% Acronyms package
\ac{ECU}      % First use: Electronic Control Unit (ECU)
\ac{ECU}      % Subsequent: ECU
\acl{ECU}     % Force long form: Electronic Control Unit
\acs{ECU}     % Force short form: ECU
```

### Consistent Terminology

**Maintain consistency for:**
- Technical terms (always capitalize properly)
- Acronyms (always same letters)
- Project names (exact phrasing)
- Metric units (always same format)

**Example consistency:**
- ✓ "Intelligent Sustainable System" (always)
- ✗ "Intelligent Sustainable System" vs. "ISS system"
- ✓ "millimeter" or "mm" (pick one)
- ✗ "mm" vs. "millimeter" mixed

---

## 📐 Units & Numbers

### Formatting Units

```latex
% Correct
Distance: 5 km          (space before unit)
Voltage: 12 V
Time: 3.5 hours
Temperature: 25 °C

% Common mistakes to avoid
5km              (no space)
5 KM             (wrong case)
5km/h            (should be km·h⁻¹ or km/h)
```

### Numbers

```latex
% Small numbers (spell out)
one, two, three, ... nine

% Large numbers (numerals)
10, 11, ... 1,000, 10,000

% Decimals
3.14159, 0.005, 99.9%

% Scientific notation
$1 \times 10^6$ or $10^6$
```

---

## ✏️ Common Writing Issues

### Active vs. Passive Voice

```latex
% Avoid passive (weak)
The system was designed to manage the battery.

% Prefer active (strong)
The system manages the battery efficiently.

% Passive is acceptable for emphasis on object
The battery management system was designed following ISO 26262 standards.
```

### Clarity

```latex
% Unclear
"The module provides functionality for the interface."

% Clear
"The Bluetooth module enables wireless communication with mobile devices."
```

### Conciseness

```latex
% Wordy
"In the event that the voltage exceeds the predetermined threshold, 
the system will implement an emergency shutdown sequence."

% Concise
"When voltage exceeds the threshold, the system triggers emergency shutdown."
```

---

## 🔍 Spelling & Grammar

### Common Technical Spelling

- **Correct:** 
  - SCRUM (all caps)
  - DevOps
  - IoT (Internet of Things)
  - Wi-Fi (with hyphen)
  - Real-time (hyphenated as adjective)
  
### UK vs. US English

Choose one and maintain consistency:
- **US:** color, realize, center, ize endings
- **UK:** colour, realise, centre, ise endings

Recommended: **US English** (standard for technical documentation)

---

## 📋 Checklist: Before Submitting Section

- ✅ All placeholder text removed
- ✅ Section contains 3+ pages of content
- ✅ All subsections completed
- ✅ LaTeX syntax is valid (no compile errors)
- ✅ Tables have captions and labels
- ✅ Figures have captions and labels
- ✅ All images optimized and in `images/` folder
- ✅ References added to `references.bib`
- ✅ Cross-references use proper `\label{}` and `\ref{}`
- ✅ Terminology is consistent throughout
- ✅ Numbers and units formatted correctly
- ✅ Active voice used where possible
- ✅ Writing is clear and concise
- ✅ Spelling and grammar verified
- ✅ PDF output reviewed for appearance
- ✅ Formatting matches other sections

---

## 🎓 Examples

### Well-Formatted Section Example

```latex
\section{Circuit Design \& Electronics}

This section details the electrical design of the vehicle's power and control systems.
It encompasses schematics, PCB design, battery management, motor drive systems, 
and sensor interfaces.

\subsection{Schematics \& PCB Design}

The electrical system comprises multiple interconnected subsystems. 
As shown in Figure~\ref{fig:system-architecture}, the primary components include:

\begin{itemize}
    \item Battery Management System (BMS)
    \item Motor drive inverter
    \item Vehicle Management Unit (VMU)
    \item Sensor and actuator networks
\end{itemize}

The circuit design follows ISO 26262 functional safety standards. 
Each critical circuit includes redundancy mechanisms as detailed in Section~\ref{sec:safety}.

Table~\ref{tab:component-specs} lists the primary electrical specifications.

\begin{table}[h]
    \centering
    \caption{Primary Electrical Component Specifications}
    \label{tab:component-specs}
    \begin{tabular}{lccr}
        \toprule
        \textbf{Component} & \textbf{Voltage} & \textbf{Current} & \textbf{Power} \\
        \midrule
        Battery Pack & 400 V & 200 A & 80 kW \\
        Motor Drive & 400 V & 150 A & 60 kW \\
        Control Electronics & 12 V & 50 A & 600 W \\
        \bottomrule
    \end{tabular}
\end{table}

% Additional content follows...
```

---

## 📞 Questions or Clarifications?

Refer to README.md for more detailed information or contact the project lead.

---

**Version:** 1.0  
**Last Updated:** March 22, 2026
