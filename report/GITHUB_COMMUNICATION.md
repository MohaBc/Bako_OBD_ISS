# GitHub-Based Team Communication Guide

Best practices for collaborating on the ISS Report using GitHub's built-in communication features.

---

## 📧 Communication Overview

### Primary Channels

| Channel | Usage | Best For |
|---------|-------|----------|
| **Issues** | Track work, bugs, feature requests | Task management, planning |
| **Pull Requests** | Code review, discussions | Changes, feedback, approval |
| **Discussions** | General questions, ideas | Q&A, announcements, brainstorming |
| **Comments** | Inline feedback | Technical discussion, review |
| **Mentions** | Direct notifications | Getting someone's attention |
| **Projects** | Timeline tracking | Progress overview, milestones |

---

## 🔖 GitHub Issues

### Purpose

Track work items, assign responsibilities, and organize tasks.

### Creating an Issue

**Go to:** Repository → Issues → New Issue

**Template: Section Assignment**

```markdown
## Section Assignment Request

**Section Number:** 06
**Section Title:** Circuit Design & Electronics
**Current Status:** Not Started ⭕

## Description
Claiming this section for content development.

## Proposed Timeline
- **Start Date:** [Date]
- **Target Completion:** [Date]
- **Estimated Time:** 20 hours

## Checklist
- [ ] Ready to start writing
- [ ] Have necessary references
- [ ] No blocking dependencies

## Questions
Any clarifications needed on section content?
```

### Using Labels

**Standard Labels:**

```
section-content      - Main section writing
subsection-00        - Executive Summary related
subsection-06        - Circuit Design related

status-backlog       - Not yet started
status-in-progress   - Currently being worked on
status-review        - Waiting for review
status-done          - Completed and merged

type-feature         - New content
type-bug             - Error to fix
type-docs            - Documentation
type-enhancement     - Improvement
type-question        - Question/discussion

priority-critical    - Urgent
priority-high        - Important
priority-medium      - Normal
priority-low         - Nice to have

help-wanted          - Assistance needed
blocked              - Cannot proceed
good-first-issue     - For new contributors
```

### Issue Comments

**Best Practices:**

```markdown
# Ask Clarifying Questions
Hi @contributor-name, quick question about subsection 6.2 - 
should we include the BMS design equations? Asking because 
similar content is in Section 4.3.

# Provide Updates
Working on this section now. Expect draft PR by Friday.
Added detailed schematics to Figure 6.1.

# Link Related Issues
This relates to #45 and blocks #50 from starting.

# Use Mentions for Attention
@project-lead - need approval on technical approach for 6.3

# Reference Commits
Working on the changes from commit abc123def
```

### Issue Workflows

**Create Issue → Assign → Comment Updates → Resolve**

```
1. Create issue when starting section
2. Add label: section-content, status-in-progress
3. Self-assign or request assignment
4. Comment with progress updates
5. Post link to PR when ready for review
6. Close when PR is merged
```

---

## 🔄 Pull Requests (PRs)

### Purpose

Submit changes for review and discussion before merging.

### Creating a PR

**Basic Flow:**

1. Push your branch to GitHub
2. GitHub shows "Compare & pull request" button
3. Click it, or go to Pull Requests → New
4. Select base branch (usually `main`)
5. Write description using template
6. Request reviewers
7. Submit

### PR Description Template

```markdown
## 📝 Changes
Detailed explanation of what this PR adds or changes.

Added comprehensive content for Section 06: Circuit Design & Electronics
including schematics, PCB design, BMS design, and motor drive systems.

## 📋 Checklist
- [x] Placeholder text replaced
- [x] All subsections completed
- [x] LaTeX compiles successfully
- [x] Images added and referenced
- [x] Bibliography updated
- [x] Spell check completed
- [x] Formatting reviewed
- [x] PDF output checked

## 🔗 Related Issues
Closes #45
Relates to #48

## 📊 Statistics
- **Pages Added:** 15
- **Figures Added:** 8
- **References Added:** 12
- **Build Status:** ✅ Passing

## 👥 Reviewers
@reviewer1 @reviewer2

## 📸 Evidence
- Before: 34 pages
- After: 49 pages
- Build log: Compiled successfully

## 💬 Notes
Section 06 completes technical design phase. Ready for 
Section 07 (Mechanical Design) to proceed.
```

### Requesting Reviewers

**Best Practices:**

1. Request 2-3 reviewers for sections
2. Include technical experts for technical sections
3. Always include project lead or senior member
4. Leave comment tagging reviewers: `@reviewer1 - Please review for technical accuracy`

**Reviewer Distribution:**

- **Technical reviewer:** Verify content accuracy
- **Style reviewer:** Check formatting consistency
- **Grammar reviewer:** Verify spelling and language

### PR Comments & Discussion

**For Reviewers:**

```markdown
# Suggesting changes (polite)
I wonder if we should clarify the BMS architecture here? 
Perhaps a diagram would help readers understand the flow better.

# Requesting changes (clear)
This section needs clarification:
1. Line 234: Spell out "MCU" on first use
2. Figure 6.3: Add axis labels to graph
3. References: Add missing page numbers to citations

# Approving
✅ Looks great! Formatting is consistent, technical content is accurate, 
and the new figures really enhance understanding. Ready to merge!
```

**For Authors:**

```markdown
# Responding to feedback
Good catch! I've clarified the MCU terminology and added full names on first use. 
Updated in commit abc123def.

Re: Figure 6.3 - Added axis labels. The graph now clearly shows 
voltage vs. time relationship. Please re-review when ready.

# Requesting re-review
Addressed all feedback. Ready for another look when you have time!
```

### Conversation Resolution

GitHub can require resolution before merging. Use this feature to ensure all discussion is addressed:

1. Reviewer makes suggestion
2. Author makes changes and commits
3. Author clicks "Resolve conversation" when done
4. Reviewer confirms resolution

---

## 💬 GitHub Discussions

### Purpose

Have team conversations, Q&A, and announcements outside of specific issues/PRs.

### Creating a Discussion

**Category: Announcements**
```
Title: Section 06 Content Review Starting
Description: Hi team! Section 06 (Circuit Design) PR is now 
in review. Comments welcome on the new diagrams and technical 
approach. Expected merge by Friday.
```

**Category: General**
```
Title: Should we include cost breakdown in BOM section?
Description: Has anyone thought about adding detailed cost 
analysis to Section 13 (BOM)? Would help product managers 
understand cost drivers.
```

**Category: Ideas**
```
Title: Add workflow diagrams for Agile section
Description: The SCRUM section (12) could benefit from swim lane 
diagrams showing team responsibilities. Should I include these?
```

### Best Discussion Topics

- Announcement of milestones
- General questions about content direction
- Brainstorming ideas for sections
- Asking for feedback on approach
- Team celebrations and updates
- Schedule changes or delays

---

## ✉️ Mentions & Notifications

### Using @Mentions

**Notify specific people:**

```markdown
# Direct question to section owner
@circuit-designer - what's your timeline for completing the schematics?

# Tag multiple people
@reviewer1 @reviewer2 @technical-lead - This needs 3 eyes before merge

# Tag teams
@iss-report/circuit-team - Ready for your section review?
```

### Notification Rules

Set GitHub notification preferences to stay informed:

1. Settings → Notifications
2. Choose notification level per repository
3. Recommended: **Watch** the repository

**Notification triggers:**
- PR assigned to you
- Someone mentions you
- You're added as reviewer
- PR you created gets comments
- Issue you created gets comments

### Managing Notification Overload

Use `watch/unwatch` to control notifications:

```bash
# Get notified for everything
git watch on

# Only get notified if mentioned
git watch mentions

# Turn off all notifications
git watch off
```

---

## 📊 GitHub Projects for Progress Tracking

### Using Project Boards

**ISS Report Progress Board:**

```
Backlog          In Progress       In Review       Done
├─ Section 07    ├─ Section 06     ├─ Section 04   ├─ Section 00
├─ Section 08    ├─ Section 12     ├─ Section 05   ├─ Section 01
├─ Section 09    ├─ Appendix B     └─ Section 03   └─ Section 02
└─ ...           └─ ...
```

### Moving Issues/PRs

1. Go to project board
2. Drag card between columns
3. Or click card → status → select column

### Adding Notes

```
Note: Waiting on circuit diagrams from CAD team before 
Section 06 can be finalized. Expected delivery: [Date]
```

---

## 🔔 Key Communication Dos & Don'ts

### ✅ DO

- Use clear, professional language
- Ask clarifying questions
- Provide specific feedback with line numbers
- Acknowledge help and good work
- Update team on progress regularly
- Link related issues and PRs
- Use formatting for readability
- Mention people when you need their attention
- Thank reviewers for their time

### ❌ DON'T

- Use negative or dismissive tone
- Leave vague comments without context
- Comment on unrelated discussions
- Mention everyone unless urgent
- Leave long paragraphs without formatting
- Discuss off-topic personal matters
- Make commitments on behalf of others
- Approve without actually reviewing
- Disappear without communication

---

## 📋 Commenting Standards

### Good Comment Example

```markdown
## ✅ Looks Good!

I've reviewed Section 06 and it looks comprehensive. Here are my observations:

### What Works Well ✓
- Clear structure with proper subsections
- Technical content is accurate
- Good use of diagrams
- References are complete

### Minor Suggestions 📝
- Line 245: "ECU" should have full name on first use
  Suggest: "Electronic Control Unit (ECU)"
- Figure 6.3: Axis labels are hard to read in small font
  Suggest: Increase font size or use larger image

### Questions ❓
1. In subsection 6.4, should we mention the charging connectors?
2. Is the BMS redundancy design covered elsewhere in report?

### Status
Ready to approve pending these minor corrections. Great work!

### Estimate
Once you address these points: ✅ Approve
```

### Poor Comment Example

```
This looks bad. Fix it.
```

---

## 🚀 Communication Timeline

### Typical Issue Lifecycle

```
Day 1: Create issue
- Post: "Starting Section 06 today"
- Status: status-in-progress

Day 2-5: Progress updates
- Comment: "Completed subsections 6.1 and 6.2, working on BMS design"
- Comment: "Added 8 diagrams, need to find more references"

Day 6: Submit PR
- "PR ready for review: #[number]"
- Tag reviewers

Day 7-8: Review & feedback
- Reviewer comments: "Great work, minor fixes needed"
- Author responds: "Fixed! Please re-review"
- Reviewer approves: "✅ Looks good!"

Day 8: Merge
- PR merged
- Close issue: "Completed - merged #[number]"
- Announcement: "Section 06 complete! 🎉"
```

---

## 📞 Escalation Process

### When Communication Gets Stuck

1. **In comment:** Ask direct question with @mention
2. **Wait 24 hours:** For response
3. **Tag supervisor:** @project-lead
4. **Last resort:** Schedule video call or in-person meeting

**Example:**
```markdown
@reviewer1 - quick follow-up on the BMS architecture question from 
2 days ago. Need decision to proceed. @project-lead - Can you weigh in?

Otherwise, proceeding with approach from our meeting notes.
```

---

## 🎯 Best Practices Summary

1. **Be specific** - Reference line numbers, provide context
2. **Be kind** - Assume good intentions
3. **Be timely** - Respond within 24 hours when possible
4. **Be complete** - Include all relevant details first time
5. **Be professional** - Use formal tone appropriate for document
6. **Be organized** - Use formatting, lists, sections
7. **Be responsive** - Acknowledge feedback and address it
8. **Be transparent** - Communicate blockers and delays early

---

## 📚 Additional Resources

- [GitHub Help - Discussions](https://docs.github.com/en/discussions)
- [GitHub Help - Issues](https://docs.github.com/en/issues)
- [GitHub Help - Pull Requests](https://docs.github.com/en/pull-requests)
- [GitHub Help - Notifications](https://docs.github.com/en/notifications)
- [Markdown Guide](https://guides.github.com/features/mastering-markdown/)

---

**Last Updated:** March 22, 2026  
**Version:** 1.0
