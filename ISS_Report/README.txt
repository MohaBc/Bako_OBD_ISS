Project README
==============

This project contains the files and templates for creating a report for an internship project. Please follow the instructions below to manipulate and change the files according to your requirements.

1. File Structure:
------------------
The project consists of the following files and directories:

   - main.tex: The main LaTeX file that controls the overall structure of the report.
   - preamble.tex: The file containing the preamble, which includes packages and settings for customizing the document.
   - titlepage.tex: The file for creating the title page of the report.
   - Preliminary Files/: A directory containing preliminary section files, such as approval.tex, declaration.tex, work-term.tex, abstract.tex, acknowledgements.tex.
   - Section Files/: A directory containing section files, such as introduction.tex, methodology.tex, results.tex, etc.
   - references.bib: The BibTeX file for managing references in the IEEE style.
   - images/ : A directory to store images used in the report.

Please note that the .ttf files (CenturyGothic.ttf, CenturyGothicbold.ttf, CGOTHICBI.ttf, gothici.ttf) in the project should not be changed or modified. They are the font files required for the Century Gothic font used in the report template.

2. Modifying the Files:
-----------------------
To customize the report for your project, follow these steps:

   - Open the main.tex file in a LaTeX editor.
   - Uncomment the "\centerline" commands in all preliminary sections files to center-align the titles. Note that uncommenting these commands may prompt some error messages, but they can be ignored as they are related to the file structure.
   - Review the comments within each file to understand the specifics and make any necessary changes to the content.
   - Add your references in BibTeX style to the `references.bib` file. Make sure to follow the IEEE citation style guidelines.
   - Place any images or figures you want to include in the images/ directory.

3. Compiling the Document:
--------------------------
To compile the report, you can use any LaTeX compiler of your choice. Here's a typical compilation process:

   - Open the main.tex file in a LaTeX editor.
   - Compile the document using LuaLaTeX instead of the traditional LaTeX compiler.
   - If necessary, compile multiple times to resolve cross-references and generate the table of contents, list of figures, and list of tables.

4. Customization and Further Instructions:
------------------------------------------
For further customization or specific instructions, please refer to the comments within each file. Additionally, you can find a collection of useful LaTeX commands in the `useful_commands.tex` file. Feel free to reference this file for examples and instructions on creating tables, figures, citing sources, and referencing tables and figures within your report.

Please note that this template is designed to comply with the MedTech guidelines. However, it is always recommended to consult the MedTech Internship guide or your supervisor for any additional requirements or formatting guidelines.

If you encounter any issues or have any questions, please do not hesitate to contact me at yahya.hamdaoui@medtech.tn.

We hope this template helps you create an excellent report for your internship. Happy writing!
