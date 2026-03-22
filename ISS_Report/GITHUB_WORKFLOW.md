# GitHub Workflow Guide for ISS Report Contributors

This guide outlines the complete GitHub workflow for collaborating on the ISS Report project.

---

## 📌 Prerequisites

Before you start, ensure you have:

1. **Git installed:** [https://git-scm.com/download](https://git-scm.com/download)
2. **GitHub account:** [https://github.com/signup](https://github.com/signup)
3. **Access to repository:** Request from project lead
4. **SSH key configured:** [GitHub SSH Setup Guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh)
5. **Git configured locally:**
   ```bash
   git config --global user.name "Your Name"
   git config --global user.email "your.email@example.com"
   ```

---

## 🚀 Initial Setup

### Clone the Repository

```bash
# Clone via SSH (recommended)
git clone git@github.com:organization/iss-report.git
cd iss-report

# Or via HTTPS (if SSH not configured)
git clone https://github.com/organization/iss-report.git
cd iss-report
```

### Verify Your Setup

```bash
# Check remote URL
git remote -v

# Should show:
# origin  git@github.com:organization/iss-report.git (fetch)
# origin  git@github.com:organization/iss-report.git (push)

# Check your configuration
git config --list | grep user
```

---

## 🔄 Standard Workflow

### 1. Start New Work

**Before starting, always update your local repository:**

```bash
# Switch to main branch
git checkout main

# Fetch latest changes from remote
git fetch origin

# Update your local main branch
git pull origin main
```

### 2. Create a Feature Branch

**For Section Work:**

```bash
# Create and switch to feature branch
git checkout -b feature/section-XX-your-name

# Naming convention examples:
git checkout -b feature/section-06-circuit-design
git checkout -b feature/section-12-agile-management
git checkout -b feature/section-20-recommendations
```

**For Other Work:**

```bash
# Bug fixes
git checkout -b fix/description

# Documentation
git checkout -b docs/description

# Configuration/Setup
git checkout -b setup/description

# Examples:
git checkout -b fix/compilation-error
git checkout -b docs/readme-update
git checkout -b setup/github-workflows
```

### 3. Edit Your Section

```bash
# Edit your assigned section file
nano "Section Files/06-circuit-design.tex"

# Or use your preferred editor
code .
```

### 4. Stage Changes

```bash
# Stage specific file
git add "Section Files/06-circuit-design.tex"

# Or stage specific section files
git add Section\ Files/06-*.tex

# Or stage all changes (use carefully)
git add .

# Check what's staged
git status
```

### 5. Commit Changes

**Write meaningful commit messages:**

```bash
# Single-line commit
git commit -m "Add content to Section 06: Circuit Design & Electronics"

# Multi-line commit (recommended for larger changes)
git commit -m "Add comprehensive Circuit Design section

- Add Schematics & PCB Design subsection (3.2 pages)
- Add Battery Management System (BMS) Design subsection (2.8 pages)
- Add Motor Drive & Inverter Design subsection (2.5 pages)
- Include 5 new technical diagrams
- Add 8 new bibliography references"
```

**Commit Message Guidelines:**

```
# Format
<type>: <subject>

<body>

<footer>

# Example
feat: Add Circuit Design & Electronics section (06)

Added comprehensive electrical system design documentation:
- Schematics and PCB layout overview
- Battery Management System (BMS) design
- Motor drive and inverter specifications
- Sensor and actuator interfacing
- Design review and simulation results

This section spans 15+ pages and includes 8 technical diagrams.

Resolves #45
Related to #48
```

**Commit Type Guidelines:**
- `feat:` New section or feature
- `docs:` Documentation changes
- `fix:` Bug fixes (compilation errors, etc.)
- `style:` Formatting without content changes
- `refactor:` Restructuring without content changes
- `chore:` Build, setup, dependency updates

### 6. Push Your Branch

```bash
# Push your feature branch to GitHub
git push origin feature/section-06-circuit-design

# First time pushing a new branch
git push -u origin feature/section-06-circuit-design
```

### 7. Create a Pull Request (PR)

**On GitHub:**

1. Go to repository: https://github.com/organization/iss-report
2. You'll see a prompt: "Compare & pull request" - **click it**
3. Or navigate to "Pull requests" tab → "New pull request"
4. Ensure:
   - **Base branch:** `main`
   - **Compare branch:** `feature/section-06-circuit-design`
   - **Title:** Section 06: Circuit Design & Electronics
   - **Description:** (use template below)

### Pull Request Description Template

```markdown
## Description
Added comprehensive content for Section 06: Circuit Design & Electronics.

## Type of Change
- [x] New section content
- [ ] Bug fix
- [ ] Documentation update
- [ ] Configuration change

## Section Details
- **Section Number:** 06
- **Section Title:** Circuit Design & Electronics
- **Pages Added:** ~15
- **Figures Added:** 8
- **Tables Added:** 3
- **References Added:** 12

## Subsections Completed
- [x] 6.1 Schematics & PCB Design
- [x] 6.2 Battery Management System (BMS) Design
- [x] 6.3 Motor Drive & Inverter Design
- [x] 6.4 Charging System Design
- [x] 6.5 Sensor & Actuator Interfacing
- [x] 6.6 Design Review & Simulation Results

## Checklist
- [x] Content replaces placeholder text
- [x] All subsections completed
- [x] LaTeX compiles without errors
- [x] Images added to `images/` folder
- [x] All images referenced in text
- [x] Bibliography entries added to `references.bib`
- [x] Cross-references use proper labels
- [x] Formatting consistent with other sections
- [x] Spelling and grammar reviewed
- [x] PDF output reviewed

## Related Issues
Closes #45
Relates to #48

## Screenshots / Evidence
- PDF page count: 34 → 49 pages
- Compilation: ✓ Successful
- PDF visual review: ✓ Approved

## Additional Notes
Section follows the established formatting conventions and includes all required subsections as per the table of contents.
```

### 8. Code Review & Discussion

**Reviewers will:**
- Check content completeness
- Verify LaTeX syntax
- Review formatting consistency
- Request changes if needed

**Respond to review comments:**

```bash
# If changes requested:
# 1. Make edits to your file
git add Section\ Files/06-circuit-design.tex

# 2. Commit with reference to review
git commit -m "Address review comments for Section 06

- Clarify motor control algorithm explanation
- Add missing figure references
- Fix inconsistent terminology"

# 3. Push updated changes
git push origin feature/section-06-circuit-design

# Comment on PR: "Addressed your feedback - please re-review"
```

### 9. Merge Pull Request

**Once approved:**

1. **On GitHub:**
   - Click "Squash and merge" (recommended) or "Merge pull request"
   - Add merge commit message
   - Click "Confirm merge"

2. **Or from command line:**
   ```bash
   git checkout main
   git pull origin main
   ```

### 10. Clean Up

```bash
# Delete local feature branch
git branch -d feature/section-06-circuit-design

# Delete remote feature branch (usually auto-deleted)
git push origin --delete feature/section-06-circuit-design

# Or GitHub will offer auto-delete after merge
```

---

## 🔍 Daily Workflows

### Pulling Latest Changes

```bash
# Check current branch
git branch

# Get latest from remote
git fetch origin

# Update current branch
git pull origin main
```

### Checking Status

```bash
# View branch status
git status

# View unpushed commits
git log origin/main..HEAD

# View all branches
git branch -a
```

### Viewing Changes

```bash
# View unstaged changes
git diff

# View staged changes
git diff --staged

# View changes in a file
git diff HEAD Section\ Files/06-circuit-design.tex

# View commit history
git log --oneline -10

# View detailed commit info
git show <commit-hash>
```

---

## 🚨 Common Issues & Solutions

### Issue: "Your branch is behind by X commits"

```bash
git fetch origin
git pull origin main
```

### Issue: "Merge conflict" (when pulling)

```bash
# See conflicting files
git status

# Open file and manually resolve conflicts
# Look for: <<<<<<, ======, >>>>>>

# After resolving:
git add .
git commit -m "Resolve merge conflicts"
git push origin feature/your-branch
```

### Issue: "Need to undo last commit"

```bash
# Undo last commit but keep changes
git reset --soft HEAD~1

# Undo last commit and discard changes
git reset --hard HEAD~1

# Undo last push (use carefully!)
git reset --hard origin/main
```

### Issue: "Accidentally committed to main"

```bash
# Create new branch from current commit
git branch feature/my-section

# Reset main to previous state
git reset --hard origin/main

# Switch to your feature branch
git checkout feature/my-section
```

### Issue: "Want to see what changed in a commit"

```bash
git show <commit-hash>
git show HEAD
git show HEAD~2
```

---

## 📋 GitHub Features to Use

### Issues & Labels

**Creating an Issue:**
1. Go to "Issues" tab
2. Click "New issue"
3. Add title, description, labels, and assignees
4. Use labels: `section-content`, `bug`, `documentation`, `help-wanted`

**Referencing issues in commits:**
```bash
git commit -m "Complete Section 06

Closes #45
Relates to #48"
```

### Projects & Milestones

**Creating a Milestone:**
1. Settings → Milestones → New milestone
2. Set title: "Sections 00-05 Complete"
3. Set due date
4. Add issues to milestone

**Assign PR to milestone:**
- Open PR → Milestone dropdown → Select milestone

### Discussions

**For team discussions:**
1. Go to "Discussions" tab
2. Create discussion for questions/topics
3. Link to related issues/PRs

### Actions (CI/CD)

**Optional: Automated checks**
```yaml
# .github/workflows/latex-build.yml
name: Build LaTeX Document

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Build LaTeX
        run: |
          apt-get update
          apt-get install -y latexmk texlive-xetex texlive-latex-extra texlive-bibtex-extra
          latexmk -xelatex main.tex
```

---

## 📊 Repository Structure on GitHub

```
iss-report/
├── main (protected branch)
│   └── Only merge via PR
├── develop (optional, for integration)
├── feature/* (work branches)
├── docs/*
└── [other protected branches]
```

### Branch Protection Rules

**Recommended settings (Settings → Branches):**
- ✅ Require pull request reviews before merging (2 reviewers)
- ✅ Require status checks to pass before merging
- ✅ Require branches to be up to date before merging
- ✅ Require code reviews from code owners
- ✅ Require conversation resolution before merging
- ✅ Include administrators in restrictions

---

## 👥 Collaboration Best Practices

### Before You Start

1. ✅ Check if someone is already working on that section
2. ✅ Comment on the GitHub Issue to claim your section
3. ✅ Notify the team in Slack/Email

### During Work

1. ✅ Pull changes frequently (`git pull`)
2. ✅ Commit often with clear messages
3. ✅ Push at end of each session
4. ✅ Keep branch up to date with main
5. ✅ Don't edit files others are working on

### When Creating PR

1. ✅ Write detailed PR description
2. ✅ Link related issues
3. ✅ Request specific reviewers
4. ✅ Update PR if feedback is received
5. ✅ Respond to comments

### Code Review Etiquette

**As a reviewer:**
- Be constructive and respectful
- Approve when satisfied
- Request changes clearly
- Approve changes once addressed

**As an author:**
- Thank reviewers for feedback
- Respond to all comments
- Push updates and ask for re-review
- Don't merge until reviewed

---



## 📞 Getting Help

**GitHub Documentation:** https://docs.github.com
- [Git Handbook](https://guides.github.com/introduction/git-handbook/)
- [Hello World](https://guides.github.com/activities/hello-world/)
- [Forking Projects](https://guides.github.com/activities/forking/)
- [Understanding the GitHub Flow](https://guides.github.com/introduction/flow/)

**Command Help:**
```bash
git help <command>      # e.g., git help push
git <command> --help    # e.g., git clone --help
```

---

## 📋 Quick Reference Cheat Sheet

```bash
# Setup
git clone git@github.com:organization/iss-report.git
git config --global user.name "Your Name"

# Daily work
git checkout main
git pull origin main
git checkout -b feature/section-XX-name

# Make changes and commit
git add .
git commit -m "Descriptive message"
git push origin feature/section-XX-name

# Create PR on GitHub and wait for review

# Update based on feedback
git add .
git commit -m "Address review feedback"
git push origin feature/section-XX-name

# Merge via GitHub

# Cleanup
git branch -d feature/section-XX-name
```

---

**Last Updated:** March 22, 2026  
**Version:** 1.0
