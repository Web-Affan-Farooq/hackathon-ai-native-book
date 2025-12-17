---
description: Create a complete specification command for a feature following best practices.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

The text the user typed after `/spec-writing` in the triggering message **is** the feature description. Assume you always have it available in this conversation even if `$ARGUMENTS` appears literally below. Do not ask the user to repeat it unless they provided an empty command.

Given that feature description, do this:

1. **Ask for Feature Name**:
   - Prompt the user for the feature name for which they want to create a specification command
   - This will be used as the title for the specification

2. **Ask for Feature Description**:
   - Prompt the user for a detailed description of the feature
   - This will form the core of the specification content

3. **Ask for Destination Path**:
   - Prompt the user for the destination path where the specification command should be written
   - This tells the user where the final spec command will be saved

4. **Generate Complete Specification Command**:
   - Based on the user's inputs, generate a complete specification command following the successful pattern from the example
   - Include all necessary sections: Target audience, Focus, Success criteria, Constraints, Not building
   - Format it properly for the Docusaurus textbook structure

5. **Output the Specification Command**:
   - Present the complete specification command in a formatted way that the user can copy and use
   - Include proper markdown formatting with bash code block for the command

## General Guidelines

- Create comprehensive, clear specifications that follow the successful pattern
- Ensure all sections are properly filled with relevant content
- Use academic yet practical tone suitable for technical documentation
- Format all content for Docusaurus markdown compatibility
- Include specific success criteria that are measurable and verifiable
- Define clear boundaries between what is and isn't being built


## Example specification command for a CLI calculator :
```markdown
/sp.specify   Basic Calculator Application in Python

This module defines the specification for building a beginner-friendly calculator application using Python, focused on core programming fundamentals and clean structure.

Target audience: Beginners in Python programming and students learning basic programming logic.

Focus:
- Python syntax and program structure
- Functions for arithmetic operations (addition, subtraction, multiplication, division)
- User input handling via CLI
- Basic input validation and error handling
- Clean, readable, beginner-oriented code

Success criteria:
- Generates 5 Python files: main.py, operations.py, utils.py, errors.py, README.md
- Calculator supports:
  - Addition
  - Subtraction
  - Multiplication
  - Division (with zero-division handling)
- Program runs in terminal with a simple menu-based interface
- Code is modular and easy to extend

Constraints:
- Simple and beginner-friendly tone
- No external libraries (standard Python only)
- Clear function naming and comments
- No GUI (CLI only)
- Python 3 compatible

Not building:
- GUI calculators (Tkinter, PyQt, web-based)
- Scientific or advanced math functions
- Expression parsing (e.g., "2 + 3 * 4")
- Persistent storage or configuration files

```

## Specification Command Template

The generated specification will follow this structure: