# Quickstart Guide: AI/Spec-Driven Book Creation

## Overview
This guide will help you set up and start using the AI/Spec-Driven Book Creation workflow with Docusaurus. Follow these steps to create your first AI-assisted book.

## Prerequisites
- Node.js (v16 or higher)
- Git
- Access to AI service API (OpenAI, Anthropic, etc.)
- Basic familiarity with Markdown and Docusaurus

## Installation

### 1. Clone the Repository
```bash
git clone <your-repo-url>
cd <your-repo-directory>
```

### 2. Install Dependencies
```bash
npm install
```

### 3. Set Up Environment Variables
Create a `.env` file in the root directory:
```env
OPENAI_API_KEY=your_openai_api_key
ANTHROPIC_API_KEY=your_anthropic_api_key  # if using Anthropic
DOCS_DIR=docs  # where Docusaurus content will be stored
```

## Getting Started

### 1. Create a New Book Project
```bash
npm run create-book -- --title "Your Book Title" --author "Your Name"
```

This command will:
- Create a new book project with the specified title and author
- Set up the basic directory structure
- Initialize the Docusaurus configuration

### 2. Define Your Book Specification
Create a specification document that outlines:
- Target audience
- Content requirements
- Technical requirements
- Quality standards
- Success criteria

Example specification:
```markdown
# Book Specification: [Your Book Title]

## Target Audience
[Description of your intended readers]

## Content Requirements
- Number of chapters
- Topics to cover
- Depth of coverage
- Examples and exercises

## Technical Requirements
- Docusaurus v3
- APA citation compliance
- Accessibility standards
- SEO optimization
```

### 3. Start the Development Server
```bash
npm run dev
```

This will start the Docusaurus development server with hot reloading.

### 4. Begin Content Creation
Navigate to the book creation interface in your browser (typically http://localhost:3000) and:

1. **Create chapters**: Add chapters with titles and descriptions
2. **Add sections**: Break down chapters into logical sections
3. **Generate content**: Use AI assistance to create content blocks
4. **Add research**: Include research items with proper sources
5. **Insert citations**: Add APA-formatted citations as needed

## AI-Assisted Content Creation

### Generating Content
1. Navigate to the section where you want to add content
2. Click "Generate with AI"
3. Provide a prompt describing what content you want
4. Review and edit the generated content
5. Validate for accuracy and compliance

### Concurrent Research
1. When you need information, click "Research this topic"
2. The system will search for relevant information
3. Review the research findings
4. Incorporate findings into your content
5. Track research debt for later verification

## Quality Validation

### Running Validation Checks
```bash
npm run validate
```

This will check:
- APA citation compliance
- Specification adherence
- Content accuracy
- Accessibility standards
- SEO optimization

### Review Process
1. Run validation checks regularly
2. Address any issues flagged by the system
3. Have peers review technical content
4. Verify all citations and sources
5. Test the final build before publishing

## Building and Deployment

### Build the Book
```bash
npm run build
```

This will:
- Generate the complete Docusaurus site
- Optimize for performance
- Validate all links and references
- Prepare for deployment

### Deploy to GitHub Pages
```bash
npm run deploy
```

## Best Practices

### Content Creation
- Start with a clear specification
- Use the concurrent research approach
- Validate citations as you go
- Maintain consistent style and voice
- Regularly save and commit your work

### AI Usage
- Provide clear, specific prompts
- Review all AI-generated content
- Verify technical accuracy
- Edit for consistency with your style
- Use AI as an assistant, not the final authority

### Quality Assurance
- Run validation checks frequently
- Verify all factual claims
- Ensure APA citations are correct
- Test accessibility features
- Check cross-references and links

## Troubleshooting

### AI Integration Issues
- Verify API keys are correctly set
- Check internet connectivity
- Ensure you have sufficient API quota
- Review rate limits and adjust accordingly

### Build Issues
- Check for validation errors
- Verify all required files exist
- Ensure no broken links or references
- Review Docusaurus configuration

### Validation Failures
- Address APA citation errors first
- Verify all content meets specification
- Check for accessibility issues
- Ensure all research items are verified