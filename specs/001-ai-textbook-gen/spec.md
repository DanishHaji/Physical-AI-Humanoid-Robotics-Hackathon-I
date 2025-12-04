# Feature Specification: AI-Native Physical AI & Humanoid Robotics Textbook

**Feature Branch**: `001-ai-textbook-gen`
**Created**: 2025-12-04
**Status**: Draft
**Input**: User description: "AI-Native Physical AI & Humanoid Robotics textbook with integrated RAG chatbot - 6 chapters covering Physical AI introduction, ROS 2, Digital Twin, NVIDIA Isaac, VLA systems, and Capstone project"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Learns Core Robotics Concepts (Priority: P1)

A university student or self-learner wants to understand Physical AI and Humanoid Robotics from fundamentals to advanced applications. They access the online textbook, read through chapters sequentially, see diagrams and code examples, and practice with simulation workflows.

**Why this priority**: This is the primary use case - delivering educational content. Without this, the textbook has no value. This represents the core product.

**Independent Test**: Can be fully tested by accessing any single chapter, reading the content, viewing diagrams, and following code examples. Delivers immediate educational value even without other features.

**Acceptance Scenarios**:

1. **Given** student accesses Chapter 1, **When** they read through sections, **Then** they see clear explanations of embodied intelligence, sensor systems, and real-world applications with supporting diagrams
2. **Given** student reaches a code example in Chapter 2 (ROS 2), **When** they view the code block, **Then** they see syntax-highlighted Python code with inline comments and execution instructions
3. **Given** student completes a chapter, **When** they scroll to the bottom, **Then** they find recap questions, MCQs, and an assignment with clear requirements
4. **Given** student uses mobile device, **When** they access any chapter, **Then** content is responsive and readable without horizontal scrolling

---

### User Story 2 - Student Gets Contextual Help via RAG Chatbot (Priority: P2)

A student is confused about a specific concept (e.g., "What is URDF in ROS 2?"). They open the sidebar chatbot, ask their question, and receive an accurate answer with citations from the relevant chapter section.

**Why this priority**: Enhances learning experience but textbook is still valuable without it. This is an augmentation, not core content delivery.

**Independent Test**: Can be tested by asking 20 diverse questions across different chapters and verifying: (1) answers come only from textbook content, (2) citations are accurate, (3) response time < 3 seconds, (4) no hallucinations detected.

**Acceptance Scenarios**:

1. **Given** student is reading Chapter 2 about URDF, **When** they ask chatbot "Explain URDF for humanoids", **Then** chatbot retrieves relevant text chunks from Chapter 2, generates answer using only that context, and cites section "ROS 2 Fundamentals > URDF for humanoids"
2. **Given** student selects text about Isaac Sim, **When** they click "Ask AI about this", **Then** chatbot answers specifically about the selected text with high contextual relevance
3. **Given** student asks question outside textbook scope (e.g., "How to build a drone?"), **When** chatbot searches vector database, **Then** it responds "This topic is not covered in the textbook" without hallucinating
4. **Given** student asks in Urdu "ROS 2 kya hai?", **When** chatbot detects Urdu language, **Then** it switches to Urdu Mode and retrieves from Urdu chapter content if available

---

### User Story 3 - Instructor Reviews Student Assessment (Priority: P3)

An instructor assigns Chapter 2's capstone assessment (build ROS 2 package for humanoid control). Students submit their work. Instructor reviews using provided evaluation rubric aligned with learning outcomes.

**Why this priority**: Assessment is important for formal education settings but not critical for self-learners. Textbook delivers value without this feature.

**Independent Test**: Can be tested by completing one chapter's assessment task, checking that problem statement is clear, starter code is provided, and rubric has measurable criteria (e.g., "Node publishes at 10Hz: 2 points").

**Acceptance Scenarios**:

1. **Given** instructor assigns Chapter 2 assessment, **When** students access it, **Then** they see problem statement, requirements list, starter code template, and evaluation rubric
2. **Given** student completes assessment, **When** instructor evaluates using rubric, **Then** each criterion is measurable and unambiguous (e.g., "URDF file has 15+ links", "Launch file starts all nodes")
3. **Given** instructor wants to track cohort progress, **When** they review assessments across chapters, **Then** each assessment clearly maps to module learning objectives

---

### User Story 4 - Student Uses Personalization Feature (Priority: P4)

A student using Jetson Orin Nano on Ubuntu wants Linux-specific instructions instead of generic examples. They access chapter settings, select their hardware/OS preferences, and content adjusts to show Orin-specific commands and examples.

**Why this priority**: Optional bonus feature that enhances UX but not essential for core learning. Most students can adapt generic examples to their environment.

**Independent Test**: Can be tested by setting profile to "Jetson Orin + Ubuntu" and verifying Chapter 4 (Isaac) shows Orin-specific docker commands, memory configurations, and performance notes.

**Acceptance Scenarios**:

1. **Given** student sets preference "Jetson Orin Nano, Ubuntu 22.04, Python", **When** they read Chapter 4, **Then** code examples show Orin-specific commands (e.g., `sudo jetson_clocks`, memory optimizations)
2. **Given** student switches preference to "x86 Desktop, Windows 11, C++", **When** content re-renders, **Then** same chapter shows WSL2 setup instructions and C++ code examples
3. **Given** student's preferences stored in browser localStorage, **When** they return next day, **Then** preferences persist without re-selection

---

### User Story 5 - Non-English Speaker Reads in Urdu (Priority: P5)

A Pakistani student proficient in Urdu prefers reading technical content in their native language. They switch language to Urdu, access any chapter, and read full translations while technical terms remain in English.

**Why this priority**: Optional inclusivity feature. English version provides full value; Urdu is additive for specific demographic.

**Independent Test**: Can be tested by switching to Urdu, reading Chapter 1, and verifying: (1) all prose is translated, (2) code blocks unchanged, (3) technical terms like "ROS 2 node" remain English, (4) RTL layout applied correctly.

**Acceptance Scenarios**:

1. **Given** student clicks Urdu language switcher, **When** page reloads, **Then** all chapter text is in Urdu, navbar is RTL, and code blocks remain unchanged
2. **Given** Urdu chapter content exists, **When** chatbot receives Urdu query, **Then** it retrieves from Urdu chunks and responds in Urdu
3. **Given** technical term "URDF" appears in Urdu text, **When** student reads it, **Then** term remains as "URDF" not transliterated to Urdu script

---

### Edge Cases

- **What happens when chatbot vector database returns no results for a query?**
  System responds: "I couldn't find relevant information in the textbook for your question. Try rephrasing or check the table of contents."

- **How does system handle queries about chapters not yet written?**
  Chatbot responds: "This chapter is not yet available. Currently available chapters: 1-6."

- **What if student selects text spanning multiple concepts (e.g., ROS + Gazebo)?**
  Chatbot analyzes combined context and provides answer addressing both concepts with separate citations.

- **How does system handle very long queries (>500 words)?**
  System truncates query to first 500 words, processes normally, and warns user: "Long query detected, using first 500 words."

- **What if embedding generation fails during RAG query?**
  System logs error, returns graceful message: "Search temporarily unavailable. Please try again in a moment."

- **How does personalization handle conflicting preferences (e.g., "Jetson" but "Windows")?**
  System prioritizes hardware first, shows warning: "Jetson devices run Linux. Switching OS preference to Ubuntu."

- **What happens if Urdu translation is incomplete for a chapter?**
  System displays English version with banner: "Urdu translation in progress. Viewing English version."

- **How does system handle concurrent chatbot requests exceeding rate limit (100/hour)?**
  Returns 429 error with message: "Rate limit reached. Please wait [X] minutes before next query."

## Requirements *(mandatory)*

### Functional Requirements

**Content Generation & Structure**

- **FR-001**: System MUST generate 6 comprehensive chapters: (1) Physical AI Intro, (2) ROS 2 Fundamentals, (3) Digital Twin, (4) NVIDIA Isaac, (5) VLA Systems, (6) Capstone Project
- **FR-002**: Each chapter MUST include: Overview, Learning Outcomes, Key Concepts, Diagrams, Code Samples, Simulation Workflows, Recap Questions, MCQs, Assignments, Glossary Terms
- **FR-003**: System MUST organize content in Docusaurus-ready `/docs` folder with auto-generated sidebar navigation
- **FR-004**: All code examples MUST be syntax-highlighted with language specification (Python, YAML, XML, bash)
- **FR-005**: All code examples MUST include inline comments explaining key steps and expected outputs
- **FR-006**: Each chapter MUST include at least 3 diagrams (architecture, workflow, or concept visualizations)
- **FR-007**: Diagrams MUST be created using Mermaid syntax for Docusaurus integration
- **FR-008**: Each chapter MUST map to specific module from course structure (ROS 2 → Module 1, Digital Twin → Module 2, Isaac → Module 3, VLA → Module 4)
- **FR-009**: Content MUST maintain weekly breakdown structure (Week 1-13 embedded in chapters)

**Assessment & Learning**

- **FR-010**: Each chapter MUST include 5+ multiple-choice questions with correct answers marked
- **FR-011**: Each chapter MUST include 1 hands-on assignment with problem statement, requirements, and starter code
- **FR-012**: Each assessment MUST include evaluation rubric with measurable criteria
- **FR-013**: Capstone chapter (Chapter 6) MUST include complete system architecture diagram showing: voice input → planning → navigation → object detection → manipulation pipeline

**RAG Chatbot Backend**

- **FR-014**: System MUST implement vector database using Qdrant Cloud free tier (1GB storage limit)
- **FR-015**: System MUST store chapter metadata in Neon Serverless Postgres free tier (0.5GB storage, limited compute)
- **FR-016**: System MUST generate embeddings using OpenAI `text-embedding-3-small` model (cost-effective)
- **FR-017**: System MUST chunk content per major heading with chunk size 512-1024 tokens
- **FR-018**: Each chunk MUST include metadata: chapter number, module name, week number, concept tags, VLA tags
- **FR-019**: System MUST implement query routing logic: Explain Mode (concepts), Code Mode (ROS/Gazebo/Isaac), Urdu Mode (language detection), Exam Mode (assessment questions)
- **FR-020**: Chatbot MUST retrieve top-5 similar chunks from Qdrant before generating answer
- **FR-021**: Chatbot MUST generate answers using ONLY retrieved context (no external knowledge)
- **FR-022**: All chatbot responses MUST include citations: chapter name + section title
- **FR-023**: System MUST handle queries with response time < 3 seconds (p95 latency)

**User Interface**

- **FR-024**: System MUST deploy as static Docusaurus site on GitHub Pages
- **FR-025**: UI MUST be responsive (desktop, tablet, mobile)
- **FR-026**: Chatbot MUST be embedded in sidebar, accessible from any chapter
- **FR-027**: Users MUST be able to select text and click "Ask AI about this" for contextual queries
- **FR-028**: System MUST support light/dark mode toggle
- **FR-029**: Sidebar MUST show hierarchical structure: Modules → Chapters → Sections

**Optional Features**

- **FR-030** *(optional)*: System MAY provide Urdu translation for all chapters stored in `/docs-ur` directory
- **FR-031** *(optional)*: Urdu content MUST preserve code blocks unchanged and keep technical terms in English
- **FR-032** *(optional)*: System MAY implement personalization with user profile: name, specialization, hardware (Jetson Nano/Orin/Xavier), OS (Ubuntu/Windows), language preference (Python/C++)
- **FR-033** *(optional)*: Personalized chapters MUST conditionally render content based on stored preferences (e.g., show Jetson-specific commands for Jetson users)
- **FR-034** *(optional)*: User preferences MUST be stored in browser localStorage (no backend required)

**System Constraints**

- **FR-035**: System MUST NOT use heavy GPU processing on server (describe GPU workflows theoretically)
- **FR-036**: All cloud services MUST operate within free-tier limits (Qdrant 1GB, Neon 0.5GB, GitHub Pages unlimited)
- **FR-037**: Docusaurus build time MUST be < 5 minutes for GitHub Actions deployment
- **FR-038**: All content MUST be original and non-copyrighted
- **FR-039**: System MUST implement rate limiting: 100 chatbot requests per hour per IP address
- **FR-040**: System MUST restrict CORS to textbook domain only

### Key Entities

- **Chapter**: Represents a learning unit with title, module mapping, week number, content sections, code examples, diagrams, assessments, glossary terms. Each chapter is independently readable and contains complete information for its topic.

- **Chunk**: Represents a semantic unit of content (512-1024 tokens) extracted from chapters. Contains text content, embeddings vector, metadata (chapter ID, module, week, concept tags). Used for RAG retrieval.

- **Query**: Represents a user question to the chatbot. Contains question text, detected mode (Explain/Code/Urdu/Exam), retrieved chunk IDs, generated answer, citations. Logged for analytics.

- **User Profile** *(optional)*: Represents personalization preferences. Contains name, specialization, selected hardware (Jetson model), OS choice, language preference (Python/C++), stored in browser localStorage.

- **Assessment**: Represents a hands-on exercise per chapter. Contains problem statement, requirements list, starter code, evaluation rubric with criteria and point values. Maps to module learning objectives.

- **Diagram**: Represents a visual aid within a chapter. Contains Mermaid syntax, alt text for accessibility, caption, type (architecture/workflow/concept), chapter reference.

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Content Quality**

- **SC-001**: All 6 chapters are complete with 100% of mandatory sections filled (overview, learning outcomes, concepts, diagrams, code, workflows, assessments)
- **SC-002**: Textbook contains at least 20 diagrams total (minimum 3 per chapter) with accurate Mermaid syntax
- **SC-003**: Textbook contains at least 50 tested code examples across all chapters with syntax highlighting and comments
- **SC-004**: Each chapter contains at least 5 MCQs and 1 hands-on assignment with rubric
- **SC-005**: Capstone chapter (Chapter 6) includes complete end-to-end pipeline architecture with all integration points documented

**RAG Chatbot Performance**

- **SC-006**: Chatbot answer accuracy > 90% when manually evaluated on 100 test queries spanning all 6 chapters
- **SC-007**: Chatbot response time < 3 seconds for 95th percentile (p95) of queries
- **SC-008**: Zero hallucinations detected in 100-query test set (all answers must cite textbook content)
- **SC-009**: Chatbot correctly identifies out-of-scope questions and responds with "not covered" message (tested on 20 off-topic queries)
- **SC-010**: Query routing accuracy > 95% (correctly identifies Explain vs Code vs Urdu vs Exam mode)

**User Experience**

- **SC-011**: Lighthouse performance score > 90 for desktop and mobile
- **SC-012**: First Contentful Paint (FCP) < 1.5 seconds on 3G connection
- **SC-013**: Textbook is fully responsive on screen sizes from 320px (mobile) to 2560px (desktop) without horizontal scroll
- **SC-014**: Students can navigate from Chapter 1 to Chapter 6 using sidebar in < 5 clicks
- **SC-015**: Dark mode toggle switches theme without page reload or flicker

**Deployment & Reliability**

- **SC-016**: Textbook successfully deploys to GitHub Pages with automated GitHub Actions workflow
- **SC-017**: Docusaurus build completes in < 5 minutes
- **SC-018**: All internal links are valid (zero 404 errors when checked with link validator)
- **SC-019**: Chatbot uptime > 99% measured over 30-day rolling window
- **SC-020**: System operates within free-tier limits: Qdrant storage < 1GB, Neon storage < 0.5GB, zero infrastructure costs

**Optional Features** *(if implemented)*

- **SC-021** *(optional)*: Urdu translation covers 100% of chapters with accurate translations reviewed by native speaker
- **SC-022** *(optional)*: Personalization feature correctly renders content variants for 3 test profiles (Jetson Orin + Python, Desktop + C++, Jetson Nano + Python)
- **SC-023** *(optional)*: User preferences persist across browser sessions using localStorage

**Learning Outcomes**

- **SC-024**: Students completing all 6 chapters and assessments can describe the full autonomous humanoid robot pipeline from voice command to object manipulation
- **SC-025**: Students can write functional ROS 2 nodes (demonstrated via Chapter 2 assessment submissions)
- **SC-026**: Students can set up Gazebo simulation for humanoid robot (demonstrated via Chapter 3 assessment)
- **SC-027**: Students can configure NVIDIA Isaac Sim environment (demonstrated via Chapter 4 assessment)

## Scope & Boundaries *(mandatory)*

### In Scope

**Content Coverage**
- 6 comprehensive chapters: Physical AI Introduction, ROS 2 Fundamentals, Digital Twin Simulation, NVIDIA Isaac Platform, VLA Systems, Capstone Project
- All mandatory chapter sections: overview, learning outcomes, concepts, diagrams, code examples, workflows, recap questions, assignments, MCQs, glossary
- Module-week mapping: 4 modules (ROS 2, Digital Twin, Isaac, VLA) × 13 weeks embedded structure
- Code examples in Python, YAML, XML, bash with inline comments
- Diagrams created with Mermaid (architecture, workflows, concept maps)
- Assessment materials: problem statements, requirements, starter code, evaluation rubrics
- Hardware coverage: Jetson Orin/Nano, RealSense D435i, Unitree H1/G1 humanoid robots
- Software coverage: ROS 2 Humble, Gazebo Harmonic, Unity ML-Agents, NVIDIA Isaac Sim/Lab, OpenCV, PyTorch

**RAG Chatbot**
- FastAPI backend exposing `/query` endpoint
- Qdrant Cloud vector database (free tier) for embeddings storage
- Neon Serverless Postgres (free tier) for metadata and usage logs
- OpenAI `text-embedding-3-small` for cost-effective embeddings
- OpenAI GPT-4 for answer generation using retrieved context only
- Query routing: Explain Mode, Code Mode, Urdu Mode, Exam Mode
- Contextual querying: select-text → Ask AI feature
- Citation system: all responses reference chapter + section
- Rate limiting: 100 requests/hour/IP
- CORS restrictions to textbook domain

**Infrastructure**
- Docusaurus 3 static site generator
- GitHub Pages deployment with GitHub Actions CI/CD
- Responsive design (mobile, tablet, desktop)
- Light/dark mode support
- Auto-generated sidebar navigation
- Free-tier cloud services only (Qdrant, Neon, Vercel for backend)

**Optional Features** *(bonus scope)*
- Urdu translation: complete chapter translations in `/docs-ur` with RTL layout
- Personalization: user profiles with hardware/OS/language preferences stored in localStorage
- Progress tracking: mark chapters as read, track quiz scores (localStorage)

### Out of Scope

**Content Exclusions**
- Non-humanoid robotics (drones, wheeled robots, manipulator arms only)
- ROS 1 (legacy, not covered)
- Non-NVIDIA hardware acceleration (AMD ROCm, Intel oneAPI)
- Custom PCB design or electronics engineering
- Mechanical engineering (CAD, kinematics derivation, material science)
- Manufacturing processes (3D printing, CNC, injection molding)
- Business/entrepreneurship topics (funding, startups, marketing)
- Advanced mathematics proofs (linear algebra, calculus derivations beyond applied usage)

**Technical Exclusions**
- Interactive Jupyter notebooks (static site limitation)
- Video hosting (link to external YouTube only)
- Real-time collaboration features (e.g., Google Docs-style editing)
- User account system or authentication (static site, no backend user management)
- Dynamic content generation at runtime (pre-built static site)
- GPU-based processing on server (describe workflows theoretically only)
- Paid cloud services or infrastructure beyond free tiers
- Mobile app versions (web-only)

**Chatbot Exclusions**
- Answering questions outside textbook scope (strict retrieval-only policy)
- Generating custom code beyond what's in textbook
- Debugging student code submissions
- Real-time tutoring or conversational AI beyond Q&A
- Multi-turn conversations with context retention across sessions (stateless per query)
- Voice input/output (text-only interface)
- Image analysis or diagram interpretation

**Assessment Exclusions**
- Automated grading system (rubrics provided, human grading assumed)
- Plagiarism detection
- LMS integration (Canvas, Moodle, Blackboard)
- Gradebook or student progress tracking backend (optional localStorage only)
- Peer review workflows
- Live coding environments or sandboxes

### Dependencies & Assumptions

**Dependencies**
- GitHub repository for version control and GitHub Pages hosting
- Docusaurus 3.x framework for static site generation
- Node.js and npm for build tooling
- Qdrant Cloud free-tier account (1GB limit)
- Neon Serverless Postgres free-tier account (0.5GB limit)
- OpenAI API access for embeddings and GPT-4 (usage costs expected)
- Vercel or Netlify free tier for FastAPI backend deployment (alternative: GitHub Pages + separate hosting)
- Mermaid.js support in Docusaurus for diagram rendering

**Assumptions**
- Target audience: university students, self-learners, robotics professionals with basic programming knowledge (Python)
- Prerequisites: students have access to Ubuntu 22.04 (native or WSL2), basic command-line proficiency, and familiarity with Python syntax
- Students have access to either: (1) NVIDIA Jetson device (Orin/Nano/Xavier), OR (2) x86 desktop with NVIDIA GPU, OR (3) cloud GPU instances (AWS, GCP)
- Students can follow installation guides for ROS 2, Gazebo, Isaac Sim (step-by-step instructions provided but not automated)
- Content will be generated using Claude Code with human review for technical accuracy
- Diagrams will be AI-generated (Mermaid syntax) and reviewed for correctness
- Code examples will be manually tested on Ubuntu 22.04 + ROS 2 Humble before publication
- Textbook version 1.0 will focus on ROS 2 Humble (LTS release), not Rolling distribution
- OpenAI API costs for embeddings + GPT-4 queries will be acceptable for free educational resource (estimated <$50/month for moderate usage)
- Users will accept 3-second chatbot response time as reasonable for educational context
- Urdu translation (if implemented) will be machine-translated with human review, not professional translation
- Personalization feature (if implemented) will be client-side only using localStorage, no backend profile storage
- Students will provide their own hardware; textbook does not include equipment procurement guide
- Students using Mac or Windows will adapt Linux-based instructions (WSL2 guidance provided)

## Constraints *(if applicable)*

**Technical Constraints**
- **CON-001**: Static site only (GitHub Pages) - no server-side rendering, no backend user sessions, no dynamic page generation
- **CON-002**: Free-tier cloud services - Qdrant 1GB storage limit, Neon 0.5GB storage limit, minimal compute allowance
- **CON-003**: Docusaurus build time must be < 5 minutes to fit GitHub Actions free tier limits
- **CON-004**: Total repository size (including images, diagrams) should stay < 1GB for fast cloning and deployment
- **CON-005**: Embeddings must be generated once during content creation, not on-demand (to minimize OpenAI API costs)
- **CON-006**: No heavy GPU processing on server - Isaac Sim, Gazebo rendering, model training described theoretically or assumed to run on user's local hardware
- **CON-007**: Rate limiting enforced at 100 requests/hour/IP to prevent API cost overruns

**Content Constraints**
- **CON-008**: All content must be original or properly attributed (no copyright violations)
- **CON-009**: Code examples must be executable on Ubuntu 22.04 + ROS 2 Humble (tested environment)
- **CON-010**: Diagrams must render correctly in Docusaurus (Mermaid syntax compatibility required)
- **CON-011**: External links (research papers, official docs) must be stable URLs (prefer DOI, official docs, not blog posts)

**User Experience Constraints**
- **CON-012**: Mobile users may experience limited code readability due to screen size (acceptable trade-off for responsive design)
- **CON-013**: Chatbot does not support multi-turn conversations (each query is stateless)
- **CON-014**: Personalization (if implemented) is per-browser using localStorage (does not sync across devices)

**Operational Constraints**
- **CON-015**: No 24/7 monitoring or on-call support (educational project, best-effort availability)
- **CON-016**: Updates to textbook content require redeployment (static site limitation)
- **CON-017**: Chatbot downtime acceptable during Qdrant/Neon maintenance windows (free tier SLA)

## Non-Functional Requirements *(if applicable)*

**Performance**
- **NFR-001**: Page load time (FCP) < 1.5 seconds on 3G connection
- **NFR-002**: Lighthouse performance score > 90 (desktop and mobile)
- **NFR-003**: Chatbot query response time < 3 seconds (p95 latency)
- **NFR-004**: Vector search in Qdrant < 200ms
- **NFR-005**: Docusaurus build time < 5 minutes

**Scalability**
- **NFR-006**: Chatbot must handle 10 concurrent requests without degradation
- **NFR-007**: Textbook must support 1000+ daily page views without performance impact (static site, inherently scalable)
- **NFR-008**: Qdrant must store embeddings for 6 chapters (estimated 500-1000 chunks) within 1GB limit

**Reliability**
- **NFR-009**: Chatbot uptime > 99% (30-day rolling window)
- **NFR-010**: Zero broken internal links (validated before deployment)
- **NFR-011**: Graceful degradation: if chatbot backend is down, textbook content remains fully accessible

**Security**
- **NFR-012**: HTTPS enforced for all GitHub Pages traffic
- **NFR-013**: API keys stored in environment variables, never committed to repository
- **NFR-014**: CORS restricted to textbook domain only
- **NFR-015**: Rate limiting enforced to prevent abuse (100 requests/hour/IP)
- **NFR-016**: No PII collection or storage (user queries logged anonymously for analytics)

**Accessibility**
- **NFR-017**: WCAG 2.1 AA compliance minimum
- **NFR-018**: All diagrams must have alt text descriptions
- **NFR-019**: Color contrast ratios meet AA standards (4.5:1 for normal text)
- **NFR-020**: Keyboard navigation supported (tab order logical, focus indicators visible)
- **NFR-021**: Screen reader compatible (semantic HTML, ARIA labels where needed)

**Maintainability**
- **NFR-022**: Content written in Markdown for easy editing
- **NFR-023**: Docusaurus configuration documented in README
- **NFR-024**: Code examples include setup instructions and dependency versions
- **NFR-025**: Diagrams stored as Mermaid source code (text-based, version-controllable)

**Usability**
- **NFR-026**: Sidebar navigation follows logical hierarchy (Modules → Chapters → Sections)
- **NFR-027**: Search functionality available (Docusaurus built-in search)
- **NFR-028**: Breadcrumbs show current location in content hierarchy
- **NFR-029**: Next/Previous chapter navigation buttons at bottom of each page
- **NFR-030**: Chatbot interface is intuitive (single input box, clear send button, visible history)

**Localization** *(optional)*
- **NFR-031**: Urdu translation (if implemented) must support RTL layout
- **NFR-032**: Language switcher in navbar for easy toggling between English/Urdu
- **NFR-033**: Technical terms remain in English across all translations for consistency

## Open Questions *(if any)*

*Note: Per specification guidelines, limiting to maximum 3 critical clarifications. Other details use reasonable defaults documented in Assumptions section.*

1. **Content Depth vs Breadth Trade-off**
   [NEEDS CLARIFICATION: Should chapters prioritize deep technical details (e.g., full URDF specification, Isaac Sim API reference) or broader coverage with curated external links? Deep = longer chapters, more self-contained. Broad = shorter chapters, requires students to follow external docs.]

2. **Assessment Submission Mechanism**
   [NEEDS CLARIFICATION: How should students submit assignments - (A) No submission, self-assessment only with provided solutions, (B) GitHub Classroom integration with template repositories, (C) Manual submission via email/LMS specified in instructor's deployment? This impacts whether to include submission workflows in textbook.]

3. **Urdu Translation Scope**
   [NEEDS CLARIFICATION: For Urdu translation feature (optional), should we translate (A) All 6 chapters fully, (B) Only Chapters 1-2 (intro + ROS fundamentals) as MVP with plan to expand later, (C) Key sections only (overviews, learning outcomes, summaries) with full English content linked? This impacts initial translation workload and timeline.]

---

**Specification Quality Notes**

This specification is complete and ready for planning with the following quality confirmations:
- All mandatory sections filled with concrete, testable requirements
- Success criteria are measurable and technology-agnostic
- Requirements avoid implementation details (no framework/language specifics in requirements, only in examples/context)
- User scenarios cover primary learning flows with clear acceptance criteria
- Edge cases identified and handled
- Scope boundaries explicitly defined (in/out of scope)
- Constraints and dependencies documented
- Open questions limited to 3 critical clarifications (content depth, assessment submission, Urdu scope)

**Next Steps**: Run `/sp.clarify` to resolve open questions, then `/sp.plan` to create implementation architecture.
