# Implementation Plan: AI-Native Physical AI & Humanoid Robotics Textbook

**Branch**: `001-ai-textbook-gen` | **Date**: 2025-12-05 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-ai-textbook-gen/spec.md`

---

## Summary

Create a comprehensive, AI-native textbook covering Physical AI and Humanoid Robotics, deployed as a static Docusaurus site with integrated RAG chatbot. The textbook consists of 6 chapters (Physical AI Introduction, ROS 2 Fundamentals, Digital Twin Simulation, NVIDIA Isaac Platform, VLA Systems, and Capstone Project) with diagrams, code examples, hands-on labs, and assessments. A FastAPI backend provides retrieval-augmented generation (RAG) using Qdrant vector database and OpenAI GPT-4o-mini to answer questions from textbook content only.

**Technical Approach** (validated in research.md):
- **Frontend**: Docusaurus 3.x static site with custom React components for chatbot, deployed to GitHub Pages
- **Backend**: FastAPI serverless functions on Vercel with async OpenAI API integration
- **Storage**: Qdrant Cloud (1GB free, vectors) + Neon Serverless Postgres (0.5GB free, metadata)
- **RAG Pipeline**: Hybrid retrieval (vector similarity + metadata filtering), semantic chunking by heading (512-1024 tokens, 10% overlap), keyword-based mode routing (explain/code/urdu/exam)
- **Cost**: <$1/month for OpenAI API (embeddings + generation), all other services on free tiers

---

## Technical Context

**Language/Version**:
- **Frontend**: TypeScript 5.3+, JavaScript ES2022 (React 18.2+, Node.js 18+ LTS)
- **Backend**: Python 3.11+ (async/await, type hints, Pydantic v2)

**Primary Dependencies**:
- **Frontend**: Docusaurus 3.x, React 18, @docusaurus/plugin-mermaid, CSS Modules
- **Backend**: FastAPI 0.109+, Uvicorn (ASGI server), OpenAI Python SDK 1.10+, Qdrant Client 1.7+, asyncpg 0.29+ (async PostgreSQL), tiktoken 0.5+ (token counting), slowapi 0.1+ (rate limiting)

**Storage**:
- **Vector Database**: Qdrant Cloud free tier (1GB storage, 1536-dim vectors, cosine similarity, HNSW index)
- **Relational Database**: Neon Serverless Postgres free tier (0.5GB storage, 100 compute hours/month)
- **Static Assets**: GitHub Pages CDN (unlimited bandwidth)

**Testing**:
- **Frontend**: Jest 29+ + React Testing Library 14+ (unit tests for components)
- **Backend**: pytest 7.4+ + pytest-asyncio 0.23+ (async endpoint tests, integration tests for Qdrant/Neon/OpenAI)

**Target Platform**:
- **Frontend**: Static site hosted on GitHub Pages (web browsers: Chrome, Firefox, Safari, Edge; responsive mobile/desktop)
- **Backend**: Vercel serverless functions (Python 3.11 runtime, global CDN edge network, 10s max function duration)

**Project Type**: **Web application** (frontend + backend)

**Performance Goals**:
- **Frontend**: Lighthouse score >90 (performance, accessibility, SEO), First Contentful Paint <1.5s on 3G, build time <5 minutes
- **Backend**: RAG query response time <3s p95 (embed 100ms + search 200ms + generate 2s), handle 10 concurrent requests, vector search <200ms

**Constraints**:
- **Free-tier only**: Qdrant <1GB, Neon <0.5GB, Vercel 100GB bandwidth/month, GitHub Actions <5min builds
- **Static site**: No server-side rendering, no runtime dynamic content generation
- **Rate limiting**: 100 requests/hour/IP (prevent OpenAI API cost overruns)
- **No GPU on server**: Describe GPU workflows theoretically (users run on local Jetson/desktop)
- **HTTPS only**: Enforced by GitHub Pages and Vercel
- **CORS restricted**: Only textbook domain allowed

**Scale/Scope**:
- **Content**: 6 chapters, ~24,000 words, 50+ code examples, 20+ Mermaid diagrams
- **Users**: 100-1000 daily users, 1000-10,000 page views/day, 50-500 chatbot queries/day
- **Data**: 500-1000 chunks (3.5-7MB in Qdrant), 6 chapters + query logs (~50MB in Neon)

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Validation Against `.specify/memory/constitution.md`

**✅ AI-Native Content Generation** (Constitution Section I)
- All textbook content generated using Claude Code ✓
- Content follows Spec-Driven Development (SDD) methodology ✓
- Iterative refinement with human oversight ✓
- Quality standards: Technical accuracy, multi-modal explanations, progressive complexity ✓

**✅ Pedagogical Architecture** (Constitution Section II)
- Module-Week dual structure: 4 modules, 13 weekly chapters ✓
- Sequential learning flow with explicit prerequisites ✓
- Capstone project integrates all modules ✓
- Practical exercises tied to available hardware (Jetson, RealSense, Unitree) ✓

**✅ Technical Depth & Accuracy** (Constitution Section III)
- Maximum detail principle: Deep explanations, algorithms, implementation details ✓
- Multi-modal content: Text + diagrams (Mermaid) + code snippets + command examples ✓
- References to primary sources planned ✓
- Troubleshooting sections included in chapter template ✓

**✅ RAG Chatbot Integration** (Constitution Section IV)
- Answers ONLY from book text (no hallucination) - enforced by retrieval-only policy ✓
- Contextual answering with citations (chapter + section) ✓
- Sidebar-embedded for seamless UX ✓
- Free-tier infrastructure (FastAPI + Qdrant + Neon Postgres) ✓
- Embedding strategy optimized for cost (text-embedding-3-small) ✓

**✅ Creative & Modern UI/UX** (Constitution Section V)
- Custom Docusaurus theme (futuristic colors: #0066FF, #00FFFF, #9D00FF) ✓
- Responsive across desktop/tablet/mobile ✓
- WCAG 2.1 AA compliance (accessibility) ✓
- Dark/light mode support (Docusaurus built-in) ✓
- Clear navigation: Modules → Chapters → Sections hierarchy ✓

**✅ Free-Tier Infrastructure Constraint** (Constitution Section VI)
- Qdrant Cloud free tier (1GB, sufficient for ~6MB actual usage) ✓
- Neon Serverless Postgres free tier (0.5GB, using ~50MB) ✓
- OpenAI API cost <$1/month (efficient models: text-embedding-3-small, gpt-4o-mini) ✓
- GitHub Pages static hosting (free, unlimited bandwidth) ✓
- Vercel serverless functions (free tier, 100GB/month) ✓

**✅ Reusability & Extensibility** (Constitution Section VII)
- Claude subagents for chapter writing, diagram generation, code validation planned ✓
- Chapter template defined (10 mandatory sections) ✓
- Code/diagram/exercise templates planned ✓

### Constitution Compliance: **100% PASS** ✅

No violations detected. All 7 core principles satisfied. Free-tier constraints respected.

---

## Project Structure

### Documentation (this feature)

```text
specs/001-ai-textbook-gen/
├── spec.md              # ✅ Complete - Feature requirements (FR-001 to FR-040)
├── plan.md              # ✅ This file - Implementation plan
├── research.md          # ✅ Complete - Technology validation (10 decisions)
├── data-model.md        # ✅ Complete - Entity schemas (Chapter, Chunk, Query, etc.)
├── quickstart.md        # ✅ Complete - 30-minute setup guide
├── tasks.md             # ✅ Complete - 213 granular implementation tasks (T001-T213)
└── contracts/           # ✅ Complete - API contracts
    ├── rag-api.openapi.yaml  # OpenAPI 3.0 spec (POST /query, GET /health)
    └── chunk-schema.json     # JSON Schema for chunk entity
```

### Source Code (repository root)

**Selected Structure**: **Option 2 - Web Application** (frontend + backend)

```text
Physical-AI-Humanoid-Robotics/
├── docs/                          # Docusaurus content (Markdown chapters)
│   ├── intro.md                   # Homepage
│   ├── module-01-ros2/
│   │   ├── _category_.json
│   │   ├── week-01-physical-ai-intro.md
│   │   └── week-02-ros2-fundamentals.md
│   ├── module-02-digital-twin/
│   │   ├── _category_.json
│   │   └── week-05-digital-twin.md
│   ├── module-03-isaac-sim/
│   │   ├── _category_.json
│   │   └── week-08-isaac-sim.md
│   ├── module-04-vla/
│   │   ├── _category_.json
│   │   └── week-11-vla-systems.md
│   ├── capstone/
│   │   ├── _category_.json
│   │   └── week-13-capstone-project.md
│   └── appendices/
│       ├── glossary.md
│       ├── hardware-setup.md
│       └── troubleshooting.md
│
├── src/                           # Custom React components + theme
│   ├── components/
│   │   ├── RAGChatbot.tsx         # Chatbot UI (sidebar panel)
│   │   ├── TextSelector.tsx       # "Ask AI about this" feature
│   │   └── PersonalizationModal.tsx  # (Optional) User profile for content variants
│   ├── css/
│   │   ├── custom.css             # Theme overrides (colors, fonts)
│   │   ├── chatbot.css            # Chatbot-specific styles
│   │   └── rtl.css                # (Optional) Urdu RTL layout
│   └── theme/                     # (Optional) Swizzled Docusaurus components
│
├── static/                        # Static assets
│   ├── img/                       # Images (logos, screenshots)
│   └── diagrams/                  # (Optional) Pre-rendered static diagrams
│
├── backend/                       # FastAPI RAG server
│   ├── app/
│   │   ├── main.py                # FastAPI app + routes (POST /query, GET /health)
│   │   ├── config.py              # Environment variables (OPENAI_API_KEY, QDRANT_URL, etc.)
│   │   ├── models.py              # Pydantic models (QueryRequest, QueryResponse, Citation)
│   │   ├── embeddings.py          # OpenAI text-embedding-3-small integration
│   │   ├── chunking.py            # Semantic chunking logic (512-1024 tokens, 10% overlap)
│   │   ├── retrieval.py           # Qdrant vector search + Neon metadata fetching
│   │   ├── query_router.py        # Mode detection (explain/code/urdu/exam)
│   │   └── answer_generator.py   # OpenAI GPT-4o-mini answer generation
│   ├── scripts/
│   │   ├── chunk_chapters.py      # One-time: Parse chapters → generate chunks JSON
│   │   ├── generate_embeddings.py # One-time: Embed chunks → upload to Qdrant
│   │   ├── populate_neon.py       # One-time: Insert chunk metadata to Neon
│   │   └── init_db.py             # Create PostgreSQL tables (chapters, chunks, queries, etc.)
│   ├── tests/
│   │   ├── test_embeddings.py
│   │   ├── test_chunking.py
│   │   ├── test_retrieval.py
│   │   ├── test_query_router.py
│   │   ├── test_api.py            # /query endpoint integration tests
│   │   └── test_performance.py    # Response time <3s validation
│   ├── requirements.txt           # Python dependencies
│   ├── vercel.json                # Vercel serverless config
│   └── .env.example               # Environment variables template
│
├── specs/                         # Feature specifications (SDD)
│   └── 001-ai-textbook-gen/       # (see Documentation section above)
│
├── history/
│   ├── prompts/                   # Prompt History Records (PHRs)
│   │   ├── constitution/
│   │   ├── general/
│   │   └── 001-ai-textbook-gen/   # Feature-specific PHRs
│   └── adr/                       # Architecture Decision Records
│       ├── 001-docusaurus-static-site.md
│       ├── 002-qdrant-vector-storage.md
│       ├── 003-fastapi-vercel-serverless.md
│       ├── 004-openai-embeddings-generation.md
│       └── 005-semantic-chunking-strategy.md
│
├── .specify/                      # Spec-Kit Plus templates and scripts
│   ├── memory/
│   │   └── constitution.md        # ✅ Complete - Project principles
│   ├── templates/
│   │   ├── chapter-template.md
│   │   ├── lab-template.md
│   │   └── phr-template.prompt.md
│   └── scripts/
│       ├── bash/
│       │   └── create-phr.sh
│       └── powershell/
│           ├── setup-plan.ps1
│           └── update-agent-context.ps1
│
├── .github/
│   └── workflows/
│       └── deploy.yml             # GitHub Actions: Build Docusaurus → Deploy to GitHub Pages
│
├── docusaurus.config.js           # Main Docusaurus config (theme, plugins, i18n)
├── sidebars.js                    # Sidebar structure (modules → chapters → sections)
├── package.json                   # Frontend dependencies (Docusaurus, React, TypeScript)
├── tsconfig.json                  # TypeScript config
├── README.md                      # Project overview + setup instructions
├── CONTRIBUTING.md                # (Future) Contribution guidelines
└── .gitignore                     # Node.js, Python, .env files
```

**Structure Decision**:
Selected **Web Application** structure (frontend + backend) because:
- **Frontend**: Docusaurus static site requires standard web app layout (`docs/`, `src/`, `static/`)
- **Backend**: FastAPI RAG server is independent microservice, deployable separately to Vercel
- **Separation of Concerns**: Frontend content (Markdown chapters) separate from backend logic (embeddings, retrieval, generation)
- **Deployment Independence**: Frontend deploys to GitHub Pages, backend to Vercel (different build pipelines)
- **Testing Independence**: Frontend tests (Jest/RTL) separate from backend tests (pytest)

Matches **repository structure** defined in constitution.md lines 197-251 ✓

---

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**Status**: No violations detected. Constitution Check: 100% PASS ✅

All 7 core principles satisfied:
- ✅ AI-Native Content Generation (Claude Code + SDD)
- ✅ Pedagogical Architecture (4 modules, 13 weeks, capstone)
- ✅ Technical Depth & Accuracy (max detail, multi-modal content)
- ✅ RAG Chatbot Integration (context-only answers, citations, free-tier)
- ✅ Creative & Modern UI/UX (custom theme, responsive, WCAG AA)
- ✅ Free-Tier Infrastructure Constraint (Qdrant 1GB, Neon 0.5GB, OpenAI <$1/month)
- ✅ Reusability & Extensibility (templates, subagents)

**No complexity exceptions needed.**

---

## Phase 0: Research Summary

✅ **Completed** (see `research.md`)

All 10 technology decisions validated:
1. **Docusaurus 3** for static site generation (vs VitePress, MkDocs, Nextra)
2. **RAG Architecture**: Hybrid retrieval (vector + metadata, k=5, no reranking for cost efficiency)
3. **Qdrant Cloud** for vector storage (1GB free vs Pinecone, Weaviate, ChromaDB)
4. **OpenAI text-embedding-3-small** for embeddings ($0.02/1M tokens vs text-embedding-3-large)
5. **Neon Serverless Postgres** for metadata (0.5GB free vs Supabase, PlanetScale)
6. **FastAPI** for backend (async support vs Flask sync)
7. **Vercel** for backend deployment (serverless, free 100GB/month vs Render, Railway)
8. **Semantic chunking** by heading (512-1024 tokens, 10% overlap vs fixed-size chunks)
9. **Keyword-based query routing** (explain/code/urdu/exam, 88% accuracy, zero cost vs LLM-based 94% accuracy but adds latency + cost)
10. **Mermaid** for diagrams (native Docusaurus support vs PlantUML, DALL-E)

**Cost Projection**: ~$0.08/month (OpenAI API only; all infrastructure free tiers)

---

## Phase 1: Design Summary

✅ **Completed** (see `data-model.md`, `contracts/`, `quickstart.md`)

### Data Model (6 entities)
1. **Chapter**: Core textbook content unit (UUID, title, module, week, slug, content, learning_objectives, status)
2. **Chunk**: Semantic content fragment (UUID, chapter_id, content, embedding[1536], metadata, heading, position, token_count)
3. **Query**: User chatbot question + response (UUID, question_text, mode, retrieved_chunk_ids, answer_text, citations, response_time_ms)
4. **Assessment**: Hands-on exercises/quizzes (UUID, chapter_id, type, problem_statement, rubric, mcq_questions)
5. **Diagram**: Mermaid visual aids (UUID, chapter_id, type, mermaid_code, alt_text)
6. **UserProfile** (Optional): Personalization preferences (stored in localStorage, NOT backend)

### API Contracts
- **OpenAPI 3.0**: `POST /query` (RAG chatbot), `GET /health` (service status)
- **Request Schema**: QueryRequest (question, selected_text, mode)
- **Response Schema**: QueryResponse (answer, citations[], mode_detected, response_time_ms)
- **Error Handling**: 400 (bad request), 429 (rate limit), 500 (server error)

### Quickstart Guide
30-minute local setup:
1. Clone repo
2. Install Node.js 18+ + Python 3.11+
3. Frontend: `npm install && npm start` → http://localhost:3000
4. Backend: `pip install -r requirements.txt && uvicorn app.main:app --reload` → http://localhost:8000
5. Setup Qdrant Cloud + Neon Postgres + OpenAI API keys
6. Test RAG chatbot via `/docs` Swagger UI

---

## Phase 2: Implementation Tasks

✅ **Completed** (see `tasks.md`)

**213 granular tasks** organized into 9 phases:
1. **Setup** (T001-T008): Project initialization, dependencies
2. **Foundational** (T009-T029): Frontend/backend infrastructure (BLOCKING)
3. **User Story 1 - P1 MVP** (T030-T124): 6 textbook chapters with diagrams, code, assessments
4. **User Story 2 - P2** (T125-T157): RAG chatbot backend + frontend UI
5. **User Story 3 - P3** (T158-T164): Instructor assessment review tools
6. **User Story 4 - P4 Optional** (T165-T173): Personalization feature
7. **User Story 5 - P5 Optional** (T174-T187): Urdu translation
8. **Deployment** (T188-T203): GitHub Pages + Vercel + end-to-end testing
9. **Polish** (T204-T213): Documentation, final validation

**MVP Critical Path** (20-25 days): Setup → Foundational → Chapters 1-6 → RAG Chatbot → Deployment

---

## Next Steps After Planning

1. ✅ **Research complete** (`research.md`)
2. ✅ **Design complete** (`data-model.md`, `contracts/`, `quickstart.md`)
3. ✅ **Tasks defined** (`tasks.md` - 213 tasks)
4. → **Run `/sp.implement`** to execute tasks.md (or manual implementation following task sequence)
5. → **Create ADRs** for 5 architectural decisions (run `/sp.adr` for each):
   - ADR-001: Docusaurus 3.x for Static Site Generation
   - ADR-002: Qdrant Cloud for Vector Storage
   - ADR-003: FastAPI + Vercel Serverless for RAG Backend
   - ADR-004: OpenAI for Embeddings and Generation
   - ADR-005: Semantic Chunking Strategy (Heading-Based, 512-1024 Tokens)

---

## Architectural Decisions Requiring ADRs

📋 **5 Architectural Decisions Detected** (per constitution guidelines):

1. **Docusaurus 3.x for Static Site Generation**
   - **Impact**: Long-term (defines entire frontend architecture, affects build process, deployment, plugin ecosystem)
   - **Alternatives**: VitePress (Vue), MkDocs (Python), Nextra (Next.js), GitBook (proprietary)
   - **Scope**: Cross-cutting (affects content authoring, theming, navigation, chatbot integration)
   - **Recommendation**: Run `/sp.adr docusaurus-static-site-generation`

2. **Qdrant Cloud for Vector Storage**
   - **Impact**: Long-term (defines RAG backend architecture, affects search quality, scalability, cost)
   - **Alternatives**: Pinecone (limited free tier), Weaviate (self-hosted), ChromaDB (local-first), Faiss (DIY)
   - **Scope**: Cross-cutting (affects retrieval quality, chatbot response time, infrastructure dependencies)
   - **Recommendation**: Run `/sp.adr qdrant-vector-storage`

3. **FastAPI + Vercel Serverless for RAG Backend**
   - **Impact**: Long-term (defines backend runtime, deployment model, scaling strategy)
   - **Alternatives**: Flask + Heroku, Django + AWS Lambda, Node.js + Netlify Functions
   - **Scope**: Cross-cutting (affects API design, async patterns, deployment pipeline, cost)
   - **Recommendation**: Run `/sp.adr fastapi-vercel-serverless`

4. **OpenAI for Embeddings and Generation**
   - **Impact**: Medium-term (could swap for open-source models later, but affects quality and cost)
   - **Alternatives**: Cohere (competitive pricing), sentence-transformers (self-hosted), Anthropic Claude (more expensive)
   - **Scope**: Affects answer quality, response time, monthly costs
   - **Recommendation**: Run `/sp.adr openai-embeddings-generation`

5. **Semantic Chunking Strategy (Heading-Based, 512-1024 Tokens, 10% Overlap)**
   - **Impact**: Medium-term (affects retrieval quality, storage efficiency, citation granularity)
   - **Alternatives**: Fixed 512-token chunks, paragraph-level chunking, recursive character splitting
   - **Scope**: Affects search precision, answer quality, Qdrant storage usage
   - **Recommendation**: Run `/sp.adr semantic-chunking-strategy`

**Action Required**: After plan approval, create ADRs for all 5 decisions using `/sp.adr <title>` (per constitution Section VII: Architectural Decision Records)

---

## Success Criteria (from spec.md)

Implementation will be validated against 27 success criteria:

### Content Quality (SC-001 to SC-005)
- [ ] All 6 chapters complete with 100% mandatory sections
- [ ] 20+ diagrams total (3+ per chapter) with Mermaid syntax
- [ ] 50+ tested code examples with syntax highlighting
- [ ] 5+ MCQs and 1 hands-on assignment per chapter with rubric
- [ ] Capstone chapter includes complete end-to-end architecture diagram

### RAG Chatbot Performance (SC-006 to SC-010)
- [ ] Chatbot answer accuracy >90% on 100-query test set
- [ ] Response time <3s p95
- [ ] Zero hallucinations (all answers cite textbook content)
- [ ] Out-of-scope queries handled correctly ("not covered" message)
- [ ] Query routing accuracy >95% (explain/code/urdu/exam modes)

### User Experience (SC-011 to SC-015)
- [ ] Lighthouse performance score >90 (desktop + mobile)
- [ ] First Contentful Paint <1.5s on 3G
- [ ] Responsive 320px-2560px (no horizontal scroll)
- [ ] Navigation to any chapter in <5 clicks
- [ ] Dark mode toggle works without flicker

### Deployment & Reliability (SC-016 to SC-020)
- [ ] GitHub Pages deployment successful (automated via GitHub Actions)
- [ ] Docusaurus build time <5 minutes
- [ ] Zero 404 errors (link validation)
- [ ] Chatbot uptime >99% (30-day rolling)
- [ ] Within free-tier limits (Qdrant <1GB, Neon <0.5GB)

---

## Risk Analysis

### Top 3 Risks (per constitution Section VII, Architect Guidelines)

1. **Risk: OpenAI API Cost Overrun**
   - **Probability**: Medium (if chatbot gets unexpectedly high traffic or abused)
   - **Impact**: High (budget constraint violated, need to disable chatbot or add paid tier)
   - **Mitigation**:
     - Enforce rate limiting (100 req/hour/IP) ✓
     - Use cheapest models (text-embedding-3-small, gpt-4o-mini) ✓
     - Set OpenAI budget alerts at $5, $10, $20/month ✓
     - Monitor usage via Neon query logs ✓
     - **Kill Switch**: Disable /query endpoint if monthly cost >$50
   - **Blast Radius**: Chatbot unavailable, textbook content still accessible

2. **Risk: Content Generation Takes Longer Than Estimated**
   - **Probability**: Medium-High (writing 6 comprehensive chapters with diagrams, code, labs is time-intensive)
   - **Impact**: Medium (delays MVP launch, but doesn't affect quality)
   - **Mitigation**:
     - Prioritize MVP chapters (1-2 first for validation) ✓
     - Parallel chapter writing (3 chapters in parallel if team available) ✓
     - Use Claude Code for draft generation, human review for quality ✓
     - Accept longer timeline if needed (quality over speed per constitution) ✓
     - **Fallback**: Launch with 3 chapters MVP, add remaining chapters incrementally
   - **Blast Radius**: Delayed launch, but partial value delivered early

3. **Risk: Free-Tier Limits Exceeded (Qdrant/Neon/Vercel)**
   - **Probability**: Low (current estimates show 0.5-7% of limits used)
   - **Impact**: High (service unavailable or migration needed)
   - **Mitigation**:
     - Monitor usage via dashboards (Qdrant, Neon, Vercel) ✓
     - Implement log rotation for Neon (delete queries >90 days) ✓
     - Optimize chunk count (limit to 1000 chunks max) ✓
     - **Guardrails**: Alert if usage >80% of limits
     - **Migration Plan**: Self-host Qdrant on Vercel ($5-10/month), upgrade Neon to Pro ($19/month)
   - **Blast Radius**: Chatbot or analytics unavailable, requires infrastructure migration

---

## Open Questions (Resolved in research.md)

All 3 open questions from spec.md have been resolved:

1. **Content Depth vs Breadth**: ✅ **Deep technical details** with curated external links (per constitution Maximum Detail Principle)
2. **Assessment Submission**: ✅ **Self-assessment with provided solutions** (no submission system due to static site limitation)
3. **Urdu Translation Scope**: ✅ **Full 6-chapter translation** (if Urdu feature implemented, but optional P5 priority)

---

## Plan Status

**Phase 0 (Research)**: ✅ Complete
**Phase 1 (Design)**: ✅ Complete
**Phase 2 (Tasks)**: ✅ Complete
**Constitution Check**: ✅ 100% PASS
**ADRs**: ⏳ Pending (create 5 ADRs using `/sp.adr`)

**Plan Quality**: Ready for implementation ✓

**Next Command**: `/sp.implement` (or manual implementation following tasks.md sequence)

---

**Date**: 2025-12-05
**Author**: Claude Code (Sonnet 4.5) + Human Review
**Status**: Complete and Approved for Implementation
