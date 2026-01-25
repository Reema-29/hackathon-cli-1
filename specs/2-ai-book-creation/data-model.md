# Data Model: AI/Spec-Driven Book Creation System

## Core Entities

### Book
- **id**: Unique identifier for the book
- **title**: Title of the book
- **description**: Brief description of the book content
- **version**: Current version of the book
- **status**: Draft, Review, Published, Archived
- **created_at**: Timestamp when book was created
- **updated_at**: Timestamp of last update
- **authors**: List of authors contributing to the book
- **target_audience**: Intended audience (e.g., CS students, AI engineers)

### Chapter
- **id**: Unique identifier for the chapter
- **book_id**: Reference to the parent book
- **title**: Title of the chapter
- **order**: Sequential order within the book
- **status**: Draft, Review, Published
- **word_count**: Number of words in the chapter
- **module_type**: ROS 2, Gazebo/Unity, Isaac Sim, VLA, Capstone
- **learning_objectives**: List of objectives for the chapter

### Section
- **id**: Unique identifier for the section
- **chapter_id**: Reference to the parent chapter
- **title**: Title of the section
- **order**: Sequential order within the chapter
- **content_type**: Text, Code, Diagram, Quiz, Lab
- **content**: Markdown content of the section
- **concepts_covered**: List of concepts covered in the section

### ContentBlock
- **id**: Unique identifier for the content block
- **section_id**: Reference to the parent section
- **type**: Text, Code, Image, Table, Video, Quote
- **content**: The actual content
- **metadata**: Additional information (alt text, caption, etc.)
- **validation_status**: Verified, Needs Review, Invalid

### Citation
- **id**: Unique identifier for the citation
- **content_block_id**: Reference to the content block using the citation
- **type**: Book, Journal, Website, Documentation, Research Paper
- **apa_format**: Full APA-formatted citation
- **url**: URL if applicable
- **accessed_date**: Date the source was accessed
- **verified**: Boolean indicating if the source was verified

### ResearchItem
- **id**: Unique identifier for the research item
- **book_id**: Reference to the associated book
- **status**: Pending, In Progress, Completed, Discarded
- **topic**: Topic or concept to research
- **source_url**: URL of the research source
- **summary**: Summary of the research findings
- **relevance_score**: Score indicating how relevant the research is
- **verified**: Boolean indicating if the information was verified

### ValidationRule
- **id**: Unique identifier for the validation rule
- **type**: Content Accuracy, APA Citation, Technical Fact, Build Integrity
- **description**: Description of what the rule validates
- **severity**: Error, Warning, Info
- **active**: Whether the rule is currently active

## Relationships

- **Book** 1 → * **Chapter**: A book contains multiple chapters
- **Chapter** 1 → * **Section**: A chapter contains multiple sections
- **Section** 1 → * **ContentBlock**: A section contains multiple content blocks
- **ContentBlock** * → * **Citation**: Content blocks may reference multiple citations
- **Book** 1 → * **ResearchItem**: A book has multiple associated research items
- **ContentBlock** 1 → * **ValidationRule**: Content blocks are validated by multiple rules

## Validation Rules

### Content Accuracy Validation
- All technical claims must be verifiable
- Code examples must follow best practices
- Terminology must be consistent throughout the book

### APA Citation Compliance
- All citations must follow APA format
- Source verification must be completed
- Plagiarism check must pass

### Technical Fact Validation
- All ROS 2, Isaac Sim, Gazebo, etc. references must be accurate
- Code examples must be runnable in specified environments
- Architecture diagrams must match described implementations

### Build Integrity Validation
- All internal links must be valid
- Images and assets must be accessible
- Docusaurus build must complete without errors

## State Transitions

### Chapter States
- Draft → Review (when content is complete for review)
- Review → Draft (when changes are requested)
- Review → Published (when approved for publication)
- Published → Review (when updates are needed)

### ResearchItem States
- Pending → In Progress (when research begins)
- In Progress → Completed (when research is finished)
- In Progress → Discarded (when research is deemed irrelevant)
- Completed → In Progress (when verification fails)

## Additional Data Structures

### BookConfiguration
- **docusaurus_config**: Docusaurus-specific configuration
- **navigation_structure**: Sidebar and navigation configuration
- **deployment_settings**: Settings for GitHub Pages deployment
- **seo_metadata**: SEO-related metadata templates

### QualityMetrics
- **accuracy_score**: Overall accuracy rating
- **citation_compliance**: Percentage of compliant citations
- **technical_verification**: Percentage of verified technical content
- **user_satisfaction**: User feedback scores
- **build_success_rate**: Percentage of successful builds