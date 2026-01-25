# Quickstart Guide: AI/Spec-Driven Book Creation

## Overview

This guide will help you get started with the AI/Spec-Driven Book Creation workflow using Docusaurus. Follow these steps to create your first book based on a feature specification.

## Prerequisites

- Node.js 18+ installed
- Git installed
- Basic knowledge of Markdown and Docusaurus
- Access to the book creation API (if using hosted version)

## Setting Up Your Environment

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies

```bash
npm install
```

### 3. Set Up Environment Variables

Create a `.env` file in the root directory:

```env
# API Configuration
API_BASE_URL=https://api.book-creation.example.com/v1
API_KEY=your-api-key-here

# Docusaurus Configuration
DOCUSAURUS_PORT=3000
DOCUSAURUS_HOST=localhost

# AI Integration (if applicable)
CLAUDE_API_KEY=your-claude-api-key
```

## Creating Your First Book

### 1. Define Your Feature Specification

Create a feature specification in `specs/` directory. The specification should include:

- Target audience
- Book objectives
- Chapter outline
- Technical requirements
- Success criteria

Example spec structure:

```markdown
# Feature Specification: [Book Title]

## Target Audience
[Who is this book for?]

## Learning Objectives
[What will readers learn?]

## Chapter Outline
[Structure of the book with main topics]

## Technical Requirements
[Specific technologies, tools, or frameworks to cover]

## Success Criteria
[How will you measure the book's success?]
```

### 2. Initialize the Book Structure

Run the book creation command:

```bash
npm run create-book -- --spec-path specs/my-book/spec.md
```

This will:
- Create the book structure in `frontend/docs/`
- Generate initial chapter files
- Set up navigation in `sidebars.ts`
- Configure Docusaurus for your book

### 3. Generate Content Using AI Assistance

Use the AI content generation tools to create initial drafts:

```bash
npm run ai-generate -- --book-id my-book-id --chapter-id chapter-1
```

### 4. Validate Content

Run validation checks to ensure quality:

```bash
# Validate APA citations
npm run validate-citations

# Check technical accuracy
npm run validate-technical

# Run build validation
npm run validate-build
```

## Research-Concurrent Workflow

### 1. Add Research Items

As you write, identify areas that need research:

```bash
npm run add-research -- --book-id my-book-id --topic "ROS 2 communication patterns"
```

### 2. Track Research Progress

Monitor your research items:

```bash
npm run list-research -- --book-id my-book-id
```

### 3. Integrate Research Findings

Once research is complete, integrate findings into your content:

```bash
npm run integrate-research -- --book-id my-book-id --research-id research-123
```

## Quality Validation Process

### 1. Content Accuracy Check

Verify all technical claims:

```bash
npm run validate-accuracy
```

### 2. APA Citation Compliance

Ensure all citations follow APA format:

```bash
npm run validate-apa
```

### 3. Build Integrity Check

Test that the documentation builds correctly:

```bash
npm run build
```

### 4. Deployment Validation

Check that the site deploys correctly:

```bash
npm run deploy -- --preview
```

## Content Structure Guidelines

### Chapter Structure (per Constitution)

Each chapter should follow the structure: Concepts → Architecture → Code → Lab → Quiz

```
chapters/
├── 01-introduction/
│   ├── concepts.md
│   ├── architecture.md
│   ├── code.md
│   ├── lab.md
│   └── quiz.md
├── 02-ros2-basics/
│   ├── concepts.md
│   ├── architecture.md
│   ├── code.md
│   ├── lab.md
│   └── quiz.md
```

### Content Block Types

- **Text**: Explanatory content
- **Code**: Code examples with syntax highlighting
- **Diagram**: Visual representations
- **Quiz**: Interactive questions
- **Lab**: Hands-on exercises

## API Integration

If using the API for book management:

### 1. Create a Book

```bash
curl -X POST https://api.book-creation.example.com/v1/books \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $API_KEY" \
  -d '{
    "title": "My AI-Driven Book",
    "target_audience": "Computer Science Students",
    "description": "A comprehensive guide to AI and robotics"
  }'
```

### 2. Add a Chapter

```bash
curl -X POST https://api.book-creation.example.com/v1/books/{bookId}/chapters \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $API_KEY" \
  -d '{
    "title": "Introduction to Robotics",
    "order": 1,
    "module_type": "ros2"
  }'
```

### 3. Validate Content

```bash
curl -X POST https://api.book-creation.example.com/v1/content-blocks/{contentBlockId}/validate \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $API_KEY" \
  -d '{
    "validation_types": ["content_accuracy", "apa_citation", "technical_fact"]
  }'
```

## Deployment

### 1. Build for Production

```bash
npm run build
```

### 2. Deploy to GitHub Pages

```bash
npm run deploy
```

## Troubleshooting

### Common Issues

1. **Build fails due to invalid citations**: Run `npm run validate-citations` to identify issues
2. **API requests failing**: Check that your API key is correctly set in environment variables
3. **Content not appearing**: Verify that navigation is properly configured in `sidebars.ts`

### Getting Help

- Check the API documentation at `/api-docs`
- Review the Docusaurus documentation
- Contact support if issues persist