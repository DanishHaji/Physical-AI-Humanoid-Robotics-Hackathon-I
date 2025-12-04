# Physical AI & Humanoid Robotics — AI-Native Textbook Constitution

## Project Identity

**Project Name**: Physical AI & Humanoid Robotics — The Complete AI-Native Textbook

**Purpose**: Create a deeply detailed, modern, AI-native textbook that teaches the full Physical AI & Humanoid Robotics course. The book will be authored using Claude Code + Spec-Kit Plus and published with a creative, futuristic Docusaurus theme. It must integrate a context-aware RAG chatbot that answers only from selected or available chapter text.

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04

---

## Core Principles

### I. AI-Native Content Generation

**Claude-First Authorship**
- All textbook content must be generated and refined using Claude Code
- Content generation follows Spec-Driven Development (SDD) methodology
- Each chapter undergoes iterative refinement with Claude until pedagogically sound
- Human oversight ensures accuracy, coherence, and educational value
- Content versioning tracks AI-generated iterations and human edits

**Quality Standards**
- Technical accuracy verified against official documentation (ROS 2, NVIDIA Isaac, Gazebo, Unity)
- Explanations must be multi-modal: theory, workflow, code, diagrams
- Progressive complexity: fundamentals → intermediate → advanced
- Real-world applicability: every concept tied to practical robotics implementation
- Code snippets must be executable, tested, and production-ready

### II. Pedagogical Architecture

**Module-Week Dual Structure**
- 4 Major Modules organize by technology domain:
  - Module 1: ROS 2 Nervous System
  - Module 2: Digital Twin (Gazebo + Unity)
  - Module 3: NVIDIA Isaac AI Brain
  - Module 4: Vision-Language-Action (VLA)
- 13 Weekly chapters embedded within modules for timeline-based learning
- Each module contains clear learning objectives, prerequisites, and outcomes
- Weekly chapters follow consistent structure: theory → demo → hands-on → assessment

**Learning Flow Integrity**
- Concepts build sequentially; no forward dependencies without introduction
- Prerequisites explicitly stated at module and chapter level
- Capstone project integrates all prior learning
- Self-assessment checkpoints at module boundaries
- Practical exercises tied to available hardware (Jetson, RealSense, Unitree)

### III. Technical Depth & Accuracy

**Maximum Detail Principle**
- No surface-level explanations; dive deep into architecture, algorithms, and implementation
- Include mathematical foundations where relevant (kinematics, control theory, ML)
- Explain "why" not just "how": design rationale, trade-offs, alternatives
- Reference primary sources: research papers, official docs, GitHub repos
- Troubleshooting sections anticipate common issues and solutions

**Multi-Modal Content**
- Textual explanations with technical precision
- AI-generated diagrams (architecture, workflows, state machines)
- Code snippets with inline comments and context
- Command-line examples with expected outputs
- Visual aids: screenshots, architecture diagrams, flowcharts

### IV. RAG Chatbot Integration

**Contextual Answering**
- Chatbot answers ONLY from book text (no hallucination)
- "Select text → Ask AI" feature for contextual queries
- Sidebar-embedded for seamless learning experience
- Free-tier infrastructure: FastAPI + Qdrant + Neon Postgres
- Embedding strategy optimized for cost and accuracy

**Answer Quality Standards**
- Responses cite specific chapter/section for verification
- Cannot answer questions outside textbook scope
- Provides page/section references for deeper reading
- Maintains conversational context within chapter scope
- Fallback to "not covered in book" for out-of-scope queries

### V. Creative & Modern UI/UX

**Docusaurus Theme Customization**
- NOT Panaversity default theme; create unique, futuristic design
- Responsive across desktop, tablet, mobile
- Smooth animations and transitions (not distracting)
- Accessibility: WCAG 2.1 AA compliance minimum
- Dark/light mode support
- Clear typography hierarchy for technical content

**Navigation & Discovery**
- Sidebar structure mirrors Module-Week hierarchy
- Breadcrumbs for contextual navigation
- In-page table of contents for long chapters
- Search functionality (Docusaurus built-in + RAG)
- Next/Previous chapter navigation
- Progress tracking (optional bonus feature)

### VI. Free-Tier Infrastructure Constraint

**Cloud Services Budget**
- Qdrant Cloud: Free tier (1GB storage, sufficient for embeddings)
- Neon Serverless Postgres: Free tier (0.5GB storage, limited compute)
- OpenAI API: Minimize calls, batch processing, use efficient models
- GitHub Pages: Static site hosting (free, unlimited bandwidth)
- Vercel/Netlify (fallback): Free tier limits respected

**Cost Optimization Strategies**
- Embeddings generated once, cached in Qdrant
- RAG queries use efficient retrieval (top-k limited)
- Static site generation avoids runtime compute
- Image optimization for fast loading
- Lazy loading for non-critical resources

### VII. Reusability & Extensibility

**Claude Subagents & Skills**
- Chapter-writing subagent (follows template, ensures consistency)
- Diagram-generation subagent (Mermaid, PlantUML, or AI-generated)
- Code-validation subagent (lints, tests, validates examples)
- RAG-indexing subagent (embeds chapters, updates Qdrant)
- Translation subagent (Urdu localization)

**Template-Driven Content**
- Chapter template defines structure (must be followed)
- Code snippet template (language, description, explanation)
- Diagram template (title, type, description)
- Exercise template (objective, steps, validation)
- Assessment template (quiz, project, rubric)

---

## Content Standards

### Chapter Structure (Mandatory)

Each chapter must contain:
1. **Front Matter** (title, authors, date, module, week, learning objectives)
2. **Introduction** (hook, context, relevance, prerequisites)
3. **Theoretical Foundation** (concepts, definitions, mathematics)
4. **Technical Implementation** (architecture, setup, code walkthrough)
5. **Hands-On Lab** (step-by-step exercise with validation)
6. **Real-World Applications** (case studies, industry examples)
7. **Troubleshooting Guide** (common errors, solutions, debugging)
8. **Summary & Key Takeaways** (bullet points, concept map)
9. **Assessment** (quiz, project, or reflection questions)
10. **Further Reading** (papers, docs, tutorials, GitHub repos)

### Code Quality Standards

**Executable & Tested**
- All code snippets must be runnable in specified environment
- Tested on target platforms (Ubuntu 22.04, ROS 2 Humble, etc.)
- No pseudo-code unless explicitly marked as conceptual
- Includes setup instructions and dependencies

**Readable & Documented**
- Inline comments explain non-obvious logic
- Function/class docstrings follow language conventions (Python, C++)
- Variable names descriptive and consistent
- Style follows official guides (PEP 8, ROS 2 conventions)

**Security & Best Practices**
- No hardcoded secrets or credentials
- Environment variables for configuration
- Error handling and validation included
- Resource cleanup (files, connections, processes)

### Diagram Standards

**Types Allowed**
- Architecture diagrams (system, component, deployment)
- Workflow diagrams (sequence, activity, state machine)
- Data flow diagrams
- Network topology diagrams
- Control flow diagrams

**Generation Tools**
- Mermaid (preferred for Docusaurus integration)
- PlantUML (for complex UML)
- AI-generated (DALL-E, Midjourney) for conceptual illustrations
- Hand-drawn digitized (for unique explanations)

**Quality Requirements**
- High resolution (vector preferred)
- Clear labels and legends
- Consistent color scheme across book
- Accessible (alt text, descriptions)

---

## Technical Architecture

### Textbook Repository Structure

```
Physical-AI-Humanoid-Robotics/
├── .specify/                      # Spec-Kit Plus config
│   ├── memory/
│   │   └── constitution.md        # This file
│   ├── templates/
│   │   ├── chapter-template.md
│   │   ├── lab-template.md
│   │   └── phr-template.prompt.md
│   └── scripts/
│       └── bash/
│           └── create-phr.sh
├── docs/                          # Docusaurus content root
│   ├── module-01-ros2/
│   │   ├── week-01-introduction.md
│   │   ├── week-02-nodes-topics.md
│   │   └── ...
│   ├── module-02-digital-twin/
│   ├── module-03-isaac-sim/
│   ├── module-04-vla/
│   ├── capstone/
│   └── appendices/
├── src/                           # Custom theme & components
│   ├── components/
│   │   └── RAGChatbot.tsx
│   ├── css/
│   │   └── custom.css
│   └── theme/
├── static/                        # Images, videos, assets
│   ├── img/
│   └── diagrams/
├── backend/                       # RAG backend (FastAPI)
│   ├── app/
│   │   ├── main.py
│   │   ├── embeddings.py
│   │   ├── retrieval.py
│   │   └── models.py
│   ├── requirements.txt
│   └── Dockerfile
├── specs/                         # Feature specs (SDD)
│   ├── rag-chatbot/
│   │   ├── spec.md
│   │   ├── plan.md
│   │   └── tasks.md
│   └── chapter-writing/
├── history/
│   ├── prompts/                   # PHRs
│   │   ├── constitution/
│   │   ├── general/
│   │   └── [feature-name]/
│   └── adr/                       # ADRs
├── docusaurus.config.js
├── sidebars.js
└── package.json
```

### RAG Backend Architecture

**Components**
1. **FastAPI Server**: Exposes `/query` endpoint for chatbot
2. **Qdrant Vector DB**: Stores chapter embeddings (free tier)
3. **Neon Postgres**: Stores chapter metadata, usage logs
4. **OpenAI Embeddings**: `text-embedding-3-small` (cost-effective)
5. **Retrieval Logic**: Hybrid search (vector + keyword)

**Workflow**
1. User selects text or asks question in chatbot
2. Frontend sends query to FastAPI `/query`
3. Backend generates embedding for query
4. Qdrant returns top-k similar chunks (k=5)
5. Postgres fetches full chapter context for chunks
6. OpenAI GPT-4 generates answer using retrieved context only
7. Response includes chapter/section citations
8. Usage logged to Postgres for analytics

**Security**
- CORS restricted to textbook domain
- Rate limiting (100 requests/hour/IP)
- API key rotation policy
- No PII storage
- HTTPS only (enforced)

### Docusaurus Configuration

**Theme Customization**
- Primary color: Futuristic blue (#0066FF)
- Secondary color: Electric cyan (#00FFFF)
- Accent color: Neon purple (#9D00FF)
- Code blocks: Monokai theme
- Fonts: Inter (body), Fira Code (code)

**Plugins Required**
- `@docusaurus/plugin-content-docs`: Core docs functionality
- `@docusaurus/plugin-sitemap`: SEO
- `docusaurus-plugin-image-zoom`: Image zoom on click
- Custom RAG chatbot plugin (sidebar integration)

**Build & Deployment**
- Static site generation (SSG)
- Deploy to GitHub Pages via GitHub Actions
- Build command: `npm run build`
- Preview locally: `npm run start`

---

## Development Workflow

### Chapter Writing Process

1. **Feature Spec** (`specs/chapter-[name]/spec.md`)
   - Learning objectives
   - Prerequisite knowledge
   - Key concepts to cover
   - Hands-on lab requirements
   - Assessment criteria

2. **Planning** (`specs/chapter-[name]/plan.md`)
   - Chapter outline (sections, subsections)
   - Diagram requirements
   - Code examples needed
   - External resources to reference

3. **Task Breakdown** (`specs/chapter-[name]/tasks.md`)
   - Granular writing tasks
   - Diagram creation tasks
   - Code validation tasks
   - Review checkpoints

4. **Implementation** (Claude-driven)
   - Generate chapter content following template
   - Create diagrams and code examples
   - Integrate into Docusaurus structure
   - Local preview and iteration

5. **Review & Refinement**
   - Technical accuracy check
   - Pedagogical flow review
   - Code testing
   - Diagram clarity

6. **RAG Indexing**
   - Chunk chapter into semantic units
   - Generate embeddings
   - Upload to Qdrant
   - Verify retrieval quality

7. **PHR Creation**
   - Document chapter creation process
   - Record key decisions
   - Note challenges and solutions

### Git Workflow

**Branch Strategy**
- `main`: Production-ready textbook
- `develop`: Integration branch for completed chapters
- `feature/chapter-[name]`: Individual chapter work
- `feature/rag-backend`: RAG system development
- `feature/theme-customization`: UI/UX work

**Commit Standards**
- Conventional commits format
- `feat(chapter-X): add section on topic`
- `fix(rag): correct embedding dimension`
- `docs(readme): update setup instructions`
- `style(theme): adjust color palette`

**PR Requirements**
- Chapter content reviewed by human expert
- Code examples tested locally
- Diagrams render correctly
- No broken links or images
- Build passes (`npm run build`)

---

## Quality Gates

### Chapter Acceptance Criteria

**Content Completeness**
- [ ] All sections from template present
- [ ] Learning objectives clearly stated
- [ ] Hands-on lab included with validation steps
- [ ] Summary and key takeaways provided
- [ ] Assessment included (quiz or project)

**Technical Accuracy**
- [ ] Commands tested on target platform
- [ ] Code examples execute without errors
- [ ] Version numbers specified (ROS 2 Humble, etc.)
- [ ] External links valid and accessible
- [ ] References cite authoritative sources

**Pedagogical Quality**
- [ ] Concepts introduced in logical order
- [ ] Prerequisites explicitly stated
- [ ] Explanations clear for target audience
- [ ] Examples illustrate key concepts
- [ ] Common pitfalls addressed

**Formatting & Style**
- [ ] Markdown syntax correct
- [ ] Code blocks have language specified
- [ ] Diagrams have alt text
- [ ] Headings follow hierarchy
- [ ] Consistent terminology throughout

### RAG System Acceptance Criteria

**Retrieval Quality**
- [ ] Top-k results relevant to query (manual eval)
- [ ] Handles typos and variations gracefully
- [ ] Respects chapter boundaries for context
- [ ] Cites specific sections accurately
- [ ] No out-of-scope answers generated

**Performance**
- [ ] Query response time < 3 seconds (p95)
- [ ] Embedding generation < 500ms
- [ ] Vector search < 200ms
- [ ] Answer generation < 2 seconds

**Reliability**
- [ ] Handles concurrent requests (10 simultaneous)
- [ ] Graceful degradation on errors
- [ ] Rate limiting enforced
- [ ] Logging captures failures

---

## Constraints & Non-Goals

### Technical Constraints

**Must Adhere To**
- Static site only (GitHub Pages limitation)
- Free-tier cloud services (Qdrant, Neon, Vercel)
- No GPU processes on server (describe workflows only)
- Embedding size limit: 1GB (Qdrant free tier)
- Build time < 5 minutes (GitHub Actions timeout)

**Explicitly NOT Supported**
- Interactive Jupyter notebooks (static book only)
- Video hosting (link to YouTube, no self-hosting)
- Real-time collaboration features
- User accounts or authentication
- Dynamic content generation at runtime

### Content Scope

**In Scope**
- ROS 2 fundamentals → advanced topics
- Gazebo and Unity simulation
- NVIDIA Isaac Sim and Isaac Lab
- Vision-Language-Action models
- Hardware integration (Jetson, RealSense, Unitree)
- Sim-to-real transfer
- Capstone project guidance

**Out of Scope**
- Non-humanoid robotics (drones, wheeled robots)
- ROS 1 (legacy, not covered)
- Non-NVIDIA hardware acceleration (AMD, Intel)
- Custom PCB design for robots
- Manufacturing and mechanical engineering
- Business/entrepreneurship aspects

---

## Bonus Features (Optional)

### Urdu Translation

**Implementation**
- Parallel Urdu chapters in `docs-ur/`
- Language switcher in navbar
- Maintains same structure as English version
- RTL support in CSS
- Cultural adaptation where relevant

**Quality Standards**
- Technical terms retain English (e.g., "ROS 2 node")
- Explanations adapted to Urdu-speaking audience
- Reviewed by native Urdu speaker with technical background

### Personalize-Chapter Button

**Concept**
- User selects their hardware/software background
- Chapter content dynamically adjusted:
  - Linux/Windows/macOS-specific instructions
  - Jetson Nano/Orin/Xavier examples
  - Python/C++ code preferences
- Stored in local storage (no backend)

**Implementation**
- React component with user profile modal
- Conditional rendering of content sections
- Markdown frontmatter for variations

### Progress Tracking

**Features**
- Mark chapters as "read"
- Track quiz scores
- Capstone project checklist
- Overall completion percentage
- Local storage (no account needed)

---

## Governance

### Constitution Authority

This constitution supersedes all other project documentation and practices. Any deviation requires:
1. Documented justification
2. Team approval (if collaborative)
3. Amendment to this constitution
4. Migration plan for existing content

### Amendment Process

1. Propose change via GitHub issue
2. Discuss rationale and impact
3. Draft amendment text
4. Review by stakeholders
5. Merge to `main` with version bump
6. Update `Last Amended` date

### Compliance Enforcement

**Every PR Must Verify**
- [ ] Constitution principles followed
- [ ] Quality gates met
- [ ] No unjustified complexity added
- [ ] PHR created for significant work
- [ ] ADR suggested for architectural decisions

### Conflict Resolution

**Priority Order** (highest to lowest)
1. Technical accuracy (correctness > style)
2. Pedagogical quality (learning > aesthetics)
3. Free-tier constraints (cost > features)
4. Accessibility (inclusive > niche)
5. Performance (fast > fancy)

---

## Success Metrics

### Textbook Quality

- [ ] All 4 modules completed with 13 weekly chapters
- [ ] Capstone project fully documented
- [ ] 100+ diagrams and illustrations
- [ ] 200+ tested code examples
- [ ] Zero broken links or images at launch

### RAG Chatbot

- [ ] Answer accuracy: >90% (manual eval on 100 test queries)
- [ ] Response time: <3s (p95)
- [ ] No hallucinations detected in testing
- [ ] Uptime: >99% (30-day rolling)

### User Experience

- [ ] Lighthouse score: >90 (performance, accessibility, SEO)
- [ ] Mobile responsive on iOS/Android
- [ ] Build time: <5 minutes
- [ ] First contentful paint: <1.5s

### Deployment

- [ ] Successfully deployed to GitHub Pages
- [ ] Custom domain configured (if applicable)
- [ ] HTTPS enforced
- [ ] SEO meta tags present
- [ ] Sitemap submitted to search engines

---

## Appendix: Key Technologies

### Primary Stack

- **Frontend**: React 18, TypeScript, Docusaurus 3
- **Styling**: CSS Modules, Tailwind CSS (optional)
- **Backend**: FastAPI, Python 3.11+
- **Databases**: Qdrant (vectors), Neon Postgres (metadata)
- **AI**: OpenAI GPT-4, `text-embedding-3-small`
- **Deployment**: GitHub Pages, Vercel (backend)

### Robotics Technologies Covered

- **Framework**: ROS 2 Humble (primary), Rolling (mention)
- **Simulation**: Gazebo Harmonic, Unity ML-Agents, Isaac Sim
- **AI**: NVIDIA Isaac Lab, PyTorch, ONNX Runtime
- **Vision**: OpenCV, RealSense SDK, Depth Anything
- **Hardware**: Jetson Orin/Nano, RealSense D435i, Unitree H1/G1

### Development Tools

- **Version Control**: Git, GitHub
- **CI/CD**: GitHub Actions
- **Package Managers**: npm/yarn (frontend), pip (backend)
- **Linting**: ESLint, Prettier, Ruff
- **Testing**: Jest (frontend), pytest (backend)

---

**This constitution is a living document.** As the project evolves, so too will these principles, constraints, and standards. All amendments will be tracked, justified, and versioned.

**Ratified by**: Claude Code + Spec-Kit Plus | **Date**: 2025-12-04
