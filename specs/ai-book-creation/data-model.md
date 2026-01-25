# Data Model: AI/Spec-Driven Book Creation System

## Core Entities

### Book
**Description**: Top-level container for the entire book project
**Fields**:
- `id` (string, required): Unique identifier for the book
- `title` (string, required): Title of the book
- `author` (string, required): Author(s) of the book
- `description` (string): Brief description of the book content
- `status` (enum: draft, in-progress, review, published): Current status of the book
- `creation_date` (datetime): When the book project was created
- `last_modified` (datetime): When the book was last modified
- `specification_id` (string): Reference to the book specification
- `docusaurus_config` (object): Configuration for Docusaurus build
- `target_audience` (string): Intended audience for the book

**Validation Rules**:
- Title and author are required fields
- Status must be one of the defined enum values
- Creation date must be in the past
- Last modified date must be after or equal to creation date

### Chapter
**Description**: Major content division within a book
**Fields**:
- `id` (string, required): Unique identifier for the chapter
- `book_id` (string, required): Reference to the parent book
- `title` (string, required): Title of the chapter
- `number` (integer, required): Sequential number of the chapter
- `status` (enum: draft, in-progress, review, complete): Status of the chapter
- `content` (text): Main content of the chapter
- `metadata` (object): Additional metadata like tags, keywords
- `order` (integer): Order of the chapter within the book

**Validation Rules**:
- Title is required
- Number must be positive
- Order must be non-negative
- Status must be one of the defined enum values

### Section
**Description**: Content subdivision within a chapter
**Fields**:
- `id` (string, required): Unique identifier for the section
- `chapter_id` (string, required): Reference to the parent chapter
- `title` (string, required): Title of the section
- `level` (integer, required): Hierarchy level (1-6 for H1-H6)
- `content` (text): Content of the section
- `parent_id` (string): Reference to parent section (for nested sections)
- `order` (integer): Order of the section within the chapter/parent

**Validation Rules**:
- Title is required
- Level must be between 1 and 6
- Order must be non-negative
- Parent_id must reference an existing section if provided

### ContentBlock
**Description**: Individual content units (paragraphs, code blocks, etc.)
**Fields**:
- `id` (string, required): Unique identifier for the content block
- `section_id` (string, required): Reference to the parent section
- `type` (enum: paragraph, code, image, table, list, heading): Type of content
- `content` (text): The actual content
- `metadata` (object): Additional metadata like language for code blocks
- `order` (integer): Order within the section
- `validation_status` (enum: pending, valid, invalid): Validation status

**Validation Rules**:
- Type must be one of the defined enum values
- Order must be non-negative
- Validation status must be one of the defined enum values

### ResearchItem
**Description**: Research findings and sources used in content creation
**Fields**:
- `id` (string, required): Unique identifier for the research item
- `book_id` (string, required): Reference to the associated book
- `query` (text, required): Original research query
- `source` (string, required): Source of the information
- `credibility_score` (number): Score representing source credibility (0-1)
- `status` (enum: pending, verified, rejected): Verification status
- `related_content_ids` (array): IDs of content blocks that use this research
- `research_date` (datetime): When the research was conducted
- `verification_date` (datetime): When the research was verified

**Validation Rules**:
- Query and source are required
- Credibility score must be between 0 and 1
- Status must be one of the defined enum values
- Research date must be in the past

### Citation
**Description**: APA-formatted citations for sources used in content
**Fields**:
- `id` (string, required): Unique identifier for the citation
- `book_id` (string, required): Reference to the associated book
- `type` (enum: book, journal, webpage, conference, report): Type of source
- `authors` (array, required): Array of author names
- `title` (string, required): Title of the work
- `year` (integer, required): Publication year
- `source` (string): Journal, publisher, or website
- `url` (string): URL if available
- `doi` (string): DOI if available
- `page_range` (string): Page range for journal articles
- `validation_status` (enum: pending, valid, invalid): APA compliance status
- `content_block_id` (string): Reference to content block using this citation

**Validation Rules**:
- Authors, title, and year are required
- Year must be a valid year
- Validation status must be one of the defined enum values
- URL must be a valid URL format if provided
- DOI must follow proper DOI format if provided

### Specification
**Description**: Requirements and specifications for the book
**Fields**:
- `id` (string, required): Unique identifier for the specification
- `book_id` (string, required): Reference to the associated book
- `title` (string, required): Title of the specification
- `content` (text, required): Detailed specification requirements
- `target_audience` (string): Intended audience description
- `technical_requirements` (object): Technical requirements for the book
- `quality_standards` (object): Quality standards to be met
- `validation_criteria` (array): Criteria for validating spec compliance
- `created_date` (datetime): When the spec was created
- `last_updated` (datetime): When the spec was last updated

**Validation Rules**:
- Title and content are required
- Created date must be in the past
- Last updated must be after or equal to created date

### ValidationRule
**Description**: Rules for validating content quality and compliance
**Fields**:
- `id` (string, required): Unique identifier for the validation rule
- `name` (string, required): Name of the validation rule
- `description` (text): Description of what the rule validates
- `category` (enum: content, citation, style, accessibility, seo): Category of validation
- `severity` (enum: error, warning, info): Severity level of violations
- `validation_function` (string): Function or pattern to validate against
- `active` (boolean): Whether the rule is currently active
- `created_date` (datetime): When the rule was created

**Validation Rules**:
- Name is required
- Category and severity must be one of defined enum values
- Active must be boolean
- Created date must be in the past

### BuildConfiguration
**Description**: Configuration for Docusaurus build process
**Fields**:
- `id` (string, required): Unique identifier for the configuration
- `book_id` (string, required): Reference to the associated book
- `docusaurus_version` (string): Version of Docusaurus to use
- `plugins` (array): List of Docusaurus plugins to include
- `themes` (array): List of themes to apply
- `deployment_config` (object): Configuration for deployment
- `build_options` (object): Additional build options
- `last_updated` (datetime): When the configuration was last updated

**Validation Rules**:
- Docusaurus version must be valid
- Last updated must be in the past

## Relationships

### Book Relationships
- One Book has many Chapters
- One Book has many ResearchItems
- One Book has many Citations
- One Book has one Specification
- One Book has one BuildConfiguration

### Chapter Relationships
- One Chapter belongs to one Book
- One Chapter has many Sections

### Section Relationships
- One Section belongs to one Chapter
- One Section has many ContentBlocks
- One Section may have parent Section (for nesting)

### ContentBlock Relationships
- One ContentBlock belongs to one Section
- One ContentBlock may have one Citation (if it contains citations)

### ResearchItem Relationships
- One ResearchItem belongs to one Book
- One ResearchItem may be referenced by many ContentBlocks

### Citation Relationships
- One Citation belongs to one Book
- One Citation may be referenced by many ContentBlocks

## State Transitions

### Book Status Transitions
- draft → in-progress (when content creation begins)
- in-progress → review (when content is ready for review)
- review → in-progress (when changes are requested)
- review → published (when content is approved)

### Chapter/Section Status Transitions
- draft → in-progress (when actively being worked on)
- in-progress → review (when ready for review)
- review → in-progress (when changes requested)
- review → complete (when approved)

### ResearchItem Status Transitions
- pending → verified (when source validated)
- pending → rejected (when source found invalid)
- verified → pending (when new information emerges)

### Citation Validation Status Transitions
- pending → valid (when APA format verified)
- pending → invalid (when APA format incorrect)
- invalid → pending (when citation is corrected)