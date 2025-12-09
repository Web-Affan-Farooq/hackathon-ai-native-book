## Specification :

```markdown
/sp.specify VLA (Voice & Language Agents) Chapter Content

Target audience: Advanced students and professionals in robotics and AI
Focus: Understanding and implementing voice-to-action pipelines and cognitive planning for humanoid robotics

Success criteria:

- Covers voice-to-action mapping, cognitive planning frameworks, and example workflows
- Includes step-by-step practical guidance and exercises
- Exercises designed to reinforce learning and allow hands-on experimentation
- Content fully compatible with Markdown/Docusaurus docs folder structure

Constraints:

- Word count: 2000–3500 words
- Format: Markdown, to be generated inside ./textbook/docs/module-4-vla
- Internal links placeholders to previous modules or exercises
- Timeline: Complete within 1 week

Not building:

- Full NLP course from scratch
- Vendor-specific hardware integration outside simulation
- Deep AI model training details
```

---

## clarificatio:

```markdown
/sp.clarify My Chapter 4 specification is at specs/module-4/spec.md
Please analyze it for:

1. Ambiguous terms (e.g., "practical guidance"—how detailed? How many examples?)
2. Missing assumptions (audience prior knowledge? Python or SDK focus?)
3. Incomplete requirements (how to balance theory vs hands-on exercises? Number of exercises?)
4. Scope conflicts (overview vs implementation? Voice processing vs cognitive reasoning depth?)

What gaps should I address before planning the chapter structure?
```

---

## plan :

```markdown
/sp.plan Create: architecture sketch, section structure, practical example workflows, exercises, quality validation.
Decisions needing documentation: number of examples per topic, language choice, depth of cognitive planning coverage.
Testing strategy: validate content completeness against success criteria; exercises runnable in simulated or SDK environment.

Technical details:

- Follow phased structure: Index → Voice-to-Action → Cognitive Planning → Exercises
- Markdown formatting ready for Docusaurus
- All files to be generated in ./textbook/docs/module-4-vla
- Use internal links for cross-references to previous modules
- Ensure consistent terminology aligned with glossary
```
