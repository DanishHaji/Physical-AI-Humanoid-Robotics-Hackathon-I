# Tasks: AI-Native Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/001-ai-textbook-gen/`
**Prerequisites**: spec.md, plan.md, constitution.md
**Feature Branch**: `001-ai-textbook-gen`
**Date**: 2025-12-05

**Organization**: Tasks are grouped by user story priority (P1-P5) to enable independent implementation and testing. Each phase builds toward delivering complete, testable user stories.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4, US5)
- Exact file paths included in descriptions

## Path Conventions

This is a web application with:
- **Frontend**: `docs/` (Docusaurus content), `src/` (React components)
- **Backend**: `backend/` (FastAPI server)
- **Static assets**: `static/`
- **Config**: Root level (docusaurus.config.js, package.json)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for both frontend and backend

- [ ] T001 Create project directory structure per plan.md (docs/, src/, backend/, static/, specs/, history/)
- [ ] T002 Initialize Node.js project with Docusaurus 3.x dependencies (package.json)
- [ ] T003 [P] Initialize Python backend project with FastAPI dependencies (backend/requirements.txt)
- [ ] T004 [P] Configure ESLint and Prettier for frontend code quality
- [ ] T005 [P] Configure Ruff for Python backend code quality
- [ ] T006 Create .env.example template with required environment variables (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_CONNECTION_STRING)
- [ ] T007 Configure .gitignore for Node.js, Python, and environment files
- [ ] T008 Initialize Git repository and create initial commit with project structure

**Checkpoint**: Basic project structure ready for foundational work

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Frontend Foundation

- [ ] T009 Configure docusaurus.config.js with custom theme colors (futuristic blue #0066FF, cyan #00FFFF, purple #9D00FF)
- [ ] T010 Configure sidebars.js with module-based category structure (module-01-ros2, module-02-digital-twin, module-03-isaac-sim, module-04-vla, capstone)
- [ ] T011 Create custom CSS theme in src/css/custom.css with dark/light mode support
- [ ] T012 [P] Install and configure @docusaurus/plugin-content-docs
- [ ] T013 [P] Install and configure docusaurus-plugin-image-zoom
- [ ] T014 [P] Configure Mermaid support for diagram rendering
- [ ] T015 Create docs/intro.md homepage with textbook overview and navigation guide
- [ ] T016 Create _category_.json files for each module directory (module-01-ros2, module-02-digital-twin, module-03-isaac-sim, module-04-vla, capstone)

### Backend Foundation

- [ ] T017 Create backend/app/main.py with FastAPI app initialization and CORS configuration
- [ ] T018 Create backend/app/config.py for environment variable management
- [ ] T019 Create backend/app/models.py with Pydantic models (QueryRequest, QueryResponse, Citation, Chunk)
- [ ] T020 Setup Qdrant Cloud account and create collection "textbook-chunks" with 1536-dim vectors
- [ ] T021 Setup Neon Serverless Postgres account and create database "textbook-db"
- [ ] T022 Create database schema in Neon: chapters table (id, title, module, week, created_at, updated_at)
- [ ] T023 [P] Create database schema in Neon: chunks table (id, chapter_id, content, heading, position, token_count, metadata JSONB, created_at)
- [ ] T024 [P] Create database schema in Neon: queries table (id, question_text, mode, retrieved_chunk_ids, answer_text, citations, response_time_ms, user_ip, created_at)
- [ ] T025 Create backend/app/embeddings.py with OpenAI text-embedding-3-small integration
- [ ] T026 Create backend/app/chunking.py with semantic chunking logic (512-1024 tokens, 10% overlap, chunk by ## headings)
- [ ] T027 Create backend/app/query_router.py with mode detection logic (Explain/Code/Urdu/Exam based on keywords)
- [ ] T028 Implement rate limiting middleware (100 requests/hour/IP) in backend/app/main.py
- [ ] T029 Create backend/tests/ directory structure with test_embeddings.py, test_retrieval.py, test_api.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Student Learns Core Robotics Concepts (Priority: P1) 🎯 MVP

**Goal**: Deliver 6 comprehensive textbook chapters with diagrams, code examples, and assessments accessible via responsive Docusaurus site

**Independent Test**: Access any chapter via browser, verify: (1) content renders correctly with diagrams, (2) code blocks have syntax highlighting, (3) responsive on mobile/desktop, (4) assessments and MCQs present

### Chapter 1: Physical AI Introduction

- [ ] T030 [P] [US1] Create docs/module-01-ros2/week-01-physical-ai-intro.md with front matter (title, module: 1, week: 1, learning objectives)
- [ ] T031 [US1] Write Chapter 1 Section 1: Introduction (hook, context, relevance, prerequisites) in week-01-physical-ai-intro.md
- [ ] T032 [US1] Write Chapter 1 Section 2: Theoretical Foundation (embodied intelligence, sensor systems, actuators, control theory)
- [ ] T033 [US1] Write Chapter 1 Section 3: Technical Implementation (architecture overview, hardware components)
- [ ] T034 [US1] Create Mermaid diagram: Physical AI system architecture (sensors → processing → actuators feedback loop)
- [ ] T035 [US1] Write Chapter 1 Section 4: Hands-On Lab (setup development environment, verify installations)
- [ ] T036 [US1] Write Chapter 1 Section 5: Real-World Applications (Boston Dynamics, Tesla Optimus, Unitree H1/G1 case studies)
- [ ] T037 [US1] Write Chapter 1 Section 6: Troubleshooting Guide (common setup errors, solutions)
- [ ] T038 [US1] Write Chapter 1 Section 7: Summary & Key Takeaways (bullet points, concept map)
- [ ] T039 [US1] Write Chapter 1 Section 8: Assessment (5 MCQs on embodied AI concepts + 1 project: "Document your hardware setup")
- [ ] T040 [US1] Write Chapter 1 Section 9: Further Reading (papers on embodied AI, NVIDIA docs, ROS 2 introduction links)
- [ ] T041 [US1] Add glossary terms to Chapter 1 (Embodied Intelligence, Sensor Fusion, Actuator, Control Loop, Sim-to-Real)

### Chapter 2: ROS 2 Fundamentals

- [ ] T042 [P] [US1] Create docs/module-01-ros2/week-02-ros2-fundamentals.md with front matter (title, module: 1, week: 2-4)
- [ ] T043 [US1] Write Chapter 2 Introduction: Why ROS 2 for humanoid robotics, prerequisites (Ubuntu 22.04, basic Python)
- [ ] T044 [US1] Write Chapter 2 Theoretical Foundation: ROS 2 architecture (nodes, topics, services, actions), DDS middleware
- [ ] T045 [US1] Create Mermaid diagram: ROS 2 node communication (publisher → topic → subscriber)
- [ ] T046 [US1] Write Chapter 2 Technical Implementation: Installing ROS 2 Humble, workspace setup, colcon build system
- [ ] T047 [US1] Add Python code example: Simple publisher node (publishing to /cmd_vel topic) with inline comments
- [ ] T048 [US1] Add Python code example: Simple subscriber node (subscribing to /odom topic) with inline comments
- [ ] T049 [US1] Add YAML code example: Launch file for multiple nodes with parameter configuration
- [ ] T050 [US1] Add XML code example: URDF file for humanoid robot (15+ links, joints for Unitree H1)
- [ ] T051 [US1] Create Mermaid diagram: URDF kinematic tree for humanoid (base → torso → arms → legs hierarchy)
- [ ] T052 [US1] Write Chapter 2 Hands-On Lab: Create ROS 2 package, write publisher/subscriber nodes, launch with launch file
- [ ] T053 [US1] Write Chapter 2 Real-World Applications: Unitree H1 control architecture, RealSense integration example
- [ ] T054 [US1] Write Chapter 2 Troubleshooting: DDS discovery issues, workspace sourcing errors, URDF validation
- [ ] T055 [US1] Write Chapter 2 Summary & Key Takeaways (ROS 2 core concepts, URDF structure)
- [ ] T056 [US1] Write Chapter 2 Assessment: 5 MCQs on ROS 2 architecture + Project "Build ROS 2 package for humanoid joint control" with rubric
- [ ] T057 [US1] Add glossary terms (Node, Topic, Service, Action, URDF, Launch File, Colcon, DDS)

### Chapter 3: Digital Twin Simulation

- [ ] T058 [P] [US1] Create docs/module-02-digital-twin/week-05-digital-twin.md with front matter (title, module: 2, week: 5-7)
- [ ] T059 [US1] Write Chapter 3 Introduction: Sim-to-real gap, why digital twins for robotics, prerequisites (ROS 2 knowledge)
- [ ] T060 [US1] Write Chapter 3 Theoretical Foundation: Digital twin concept, physics simulation, rendering pipelines
- [ ] T061 [US1] Write Chapter 3 Technical Implementation Part 1: Gazebo Harmonic installation, world creation, SDF format
- [ ] T062 [US1] Add XML code example: Gazebo SDF world file with humanoid robot model and environment
- [ ] T063 [US1] Create Mermaid diagram: Gazebo architecture (physics engine → sensor simulation → ROS 2 bridge)
- [ ] T064 [US1] Add Python code example: Gazebo-ROS 2 bridge node for sensor data streaming
- [ ] T065 [US1] Write Chapter 3 Technical Implementation Part 2: Unity ML-Agents setup, humanoid agent configuration
- [ ] T066 [US1] Add Python code example: Unity ML-Agents training script with PPO algorithm for humanoid locomotion
- [ ] T067 [US1] Create Mermaid diagram: Unity ML-Agents training loop (observation → policy → action → reward)
- [ ] T068 [US1] Write Chapter 3 Hands-On Lab: Setup Gazebo simulation for Unitree H1, integrate with ROS 2, run basic control
- [ ] T069 [US1] Write Chapter 3 Real-World Applications: Sim-to-real transfer strategies, domain randomization, fine-tuning
- [ ] T070 [US1] Write Chapter 3 Troubleshooting: Gazebo rendering issues, Unity-ROS communication errors, physics instability
- [ ] T071 [US1] Write Chapter 3 Summary & Key Takeaways (Digital twin benefits, Gazebo vs Unity trade-offs)
- [ ] T072 [US1] Write Chapter 3 Assessment: 5 MCQs on simulation concepts + Project "Setup Gazebo environment for humanoid" with rubric
- [ ] T073 [US1] Add glossary terms (Digital Twin, Gazebo, Unity ML-Agents, SDF, Physics Engine, Domain Randomization, Sim-to-Real)

### Chapter 4: NVIDIA Isaac Platform

- [ ] T074 [P] [US1] Create docs/module-03-isaac-sim/week-08-isaac-sim.md with front matter (title, module: 3, week: 8-10)
- [ ] T075 [US1] Write Chapter 4 Introduction: NVIDIA Isaac ecosystem, why for humanoid AI, prerequisites (NVIDIA GPU, CUDA)
- [ ] T076 [US1] Write Chapter 4 Theoretical Foundation: Isaac Sim (Omniverse), Isaac Lab (RL framework), GPU acceleration
- [ ] T077 [US1] Write Chapter 4 Technical Implementation Part 1: Isaac Sim installation (Docker on Jetson Orin/Desktop), Omniverse setup
- [ ] T078 [US1] Add bash code example: Docker command to run Isaac Sim on Jetson Orin with GPU passthrough
- [ ] T079 [US1] Create Mermaid diagram: Isaac Sim architecture (USD scene → PhysX → rendering → ROS 2 bridge)
- [ ] T080 [US1] Add Python code example: Isaac Sim Python API script to load humanoid robot and run simulation
- [ ] T081 [US1] Write Chapter 4 Technical Implementation Part 2: Isaac Lab installation, RL environment setup for humanoid
- [ ] T082 [US1] Add Python code example: Isaac Lab RL training script for humanoid standing task using PPO
- [ ] T083 [US1] Create Mermaid diagram: Isaac Lab workflow (environment → agent → training → evaluation)
- [ ] T084 [US1] Write Chapter 4 Hands-On Lab: Load Unitree H1 in Isaac Sim, run physics simulation, export to Isaac Lab for RL training
- [ ] T085 [US1] Write Chapter 4 Real-World Applications: NVIDIA's humanoid research, Isaac Lab benchmarks, Jetson deployment
- [ ] T086 [US1] Write Chapter 4 Troubleshooting: Docker GPU access issues, USD file errors, Isaac Lab dependency conflicts
- [ ] T087 [US1] Write Chapter 4 Summary & Key Takeaways (Isaac Sim capabilities, Isaac Lab RL workflow)
- [ ] T088 [US1] Write Chapter 4 Assessment: 5 MCQs on Isaac platform + Project "Configure Isaac Sim environment for humanoid" with rubric
- [ ] T089 [US1] Add glossary terms (Isaac Sim, Isaac Lab, Omniverse, USD, PhysX, RL Environment, PPO)

### Chapter 5: Vision-Language-Action (VLA) Systems

- [ ] T090 [P] [US1] Create docs/module-04-vla/week-11-vla-systems.md with front matter (title, module: 4, week: 11-12)
- [ ] T091 [US1] Write Chapter 5 Introduction: VLA for embodied AI, vision-language models for robotics, prerequisites (ML basics)
- [ ] T092 [US1] Write Chapter 5 Theoretical Foundation: Vision models (Depth Anything, SAM), language models (LLMs), action prediction
- [ ] T093 [US1] Create Mermaid diagram: VLA pipeline (camera → vision model → LLM → action planner → robot control)
- [ ] T094 [US1] Write Chapter 5 Technical Implementation Part 1: RealSense D435i setup, depth estimation with Depth Anything
- [ ] T095 [US1] Add Python code example: RealSense camera integration with ROS 2, depth image processing
- [ ] T096 [US1] Add Python code example: Depth Anything model inference for scene understanding
- [ ] T097 [US1] Write Chapter 5 Technical Implementation Part 2: OpenAI API integration for language understanding and action planning
- [ ] T098 [US1] Add Python code example: VLA system - voice command → LLM → navigation/manipulation action
- [ ] T099 [US1] Create Mermaid diagram: Object detection and manipulation flow (detect → grasp planning → execution → feedback)
- [ ] T100 [US1] Write Chapter 5 Hands-On Lab: Setup RealSense camera, integrate depth estimation, implement voice-to-action pipeline
- [ ] T101 [US1] Write Chapter 5 Real-World Applications: OpenAI's humanoid robot research, Tesla Optimus vision system
- [ ] T102 [US1] Write Chapter 5 Troubleshooting: RealSense driver issues, OpenAI API errors, action prediction failures
- [ ] T103 [US1] Write Chapter 5 Summary & Key Takeaways (VLA components, vision-language integration)
- [ ] T104 [US1] Write Chapter 5 Assessment: 5 MCQs on VLA concepts + Project "Build vision-based object detection pipeline" with rubric
- [ ] T105 [US1] Add glossary terms (VLA, Depth Anything, SAM, RealSense, Object Detection, Grasp Planning, Vision-Language Model)

### Chapter 6: Capstone Project

- [ ] T106 [P] [US1] Create docs/capstone/week-13-capstone-project.md with front matter (title, module: capstone, week: 13)
- [ ] T107 [US1] Write Chapter 6 Introduction: Integrating all modules into autonomous humanoid system
- [ ] T108 [US1] Write Chapter 6 System Architecture: End-to-end pipeline design (voice → planning → navigation → manipulation)
- [ ] T109 [US1] Create Mermaid diagram: Complete system architecture (all components integrated - ROS 2, Isaac Sim, VLA, control)
- [ ] T110 [US1] Write Chapter 6 Implementation Roadmap: Step-by-step guide to build capstone project
- [ ] T111 [US1] Add Python code example: Main control node integrating voice input, VLA, navigation, and manipulation
- [ ] T112 [US1] Write Chapter 6 Testing & Validation: Simulation testing strategy, real-world deployment checklist
- [ ] T113 [US1] Write Chapter 6 Troubleshooting: Integration issues, performance bottlenecks, debugging strategies
- [ ] T114 [US1] Write Chapter 6 Summary: Project completion criteria, next steps in humanoid robotics
- [ ] T115 [US1] Write Chapter 6 Assessment: Final capstone project rubric (voice input: 20%, navigation: 25%, object detection: 25%, manipulation: 30%)
- [ ] T116 [US1] Add Chapter 6 Further Reading: Advanced research papers, open-source projects, career pathways

### Appendices

- [ ] T117 [P] [US1] Create docs/appendices/glossary.md with all glossary terms from Chapters 1-6 (50+ terms alphabetically sorted)
- [ ] T118 [P] [US1] Create docs/appendices/hardware-setup.md with detailed hardware procurement guide (Jetson models, RealSense, alternatives)
- [ ] T119 [P] [US1] Create docs/appendices/troubleshooting.md with consolidated troubleshooting tips across all chapters

### Frontend Polish for US1

- [ ] T120 [US1] Configure package.json build script and verify build completes in <5 minutes
- [ ] T121 [US1] Test responsive design on mobile (320px), tablet (768px), desktop (1920px) screen sizes
- [ ] T122 [US1] Run Lighthouse audit and ensure performance score >90
- [ ] T123 [US1] Validate all internal links are functional (no 404 errors)
- [ ] T124 [US1] Add alt text to all diagrams for accessibility (WCAG 2.1 AA compliance)

**Checkpoint**: At this point, User Story 1 (P1) should be fully functional - all 6 chapters accessible via responsive Docusaurus site with diagrams, code examples, and assessments

---

## Phase 4: User Story 2 - Student Gets Contextual Help via RAG Chatbot (Priority: P2)

**Goal**: Implement RAG backend with Qdrant + Neon + FastAPI, add chatbot UI to Docusaurus sidebar, enable contextual Q&A with citations

**Independent Test**: Ask chatbot 20 questions across different chapters, verify: (1) answers cite textbook content, (2) no hallucinations, (3) response time <3s, (4) out-of-scope queries handled correctly

### Backend Implementation

- [ ] T125 [P] [US2] Implement backend/app/retrieval.py with Qdrant vector search logic (top-k=5, similarity threshold)
- [ ] T126 [P] [US2] Implement backend/app/retrieval.py with Neon Postgres metadata fetching for retrieved chunks
- [ ] T127 [US2] Implement hybrid retrieval with metadata filtering (module, week, concept_tags) in backend/app/retrieval.py
- [ ] T128 [US2] Create backend/app/answer_generator.py with OpenAI GPT-4o-mini integration for answer generation
- [ ] T129 [US2] Implement citation extraction logic in backend/app/answer_generator.py (chapter + section references)
- [ ] T130 [US2] Create POST /query endpoint in backend/app/main.py with request validation (Pydantic models)
- [ ] T131 [US2] Add response time tracking and logging to /query endpoint
- [ ] T132 [US2] Implement out-of-scope detection logic (if no chunks retrieved above threshold, return "not covered" message)
- [ ] T133 [US2] Add error handling for OpenAI API failures with graceful degradation message

### Content Chunking & Embedding

- [ ] T134 [US2] Create backend/scripts/chunk_chapters.py to parse all 6 chapters and generate chunks (512-1024 tokens, 10% overlap)
- [ ] T135 [US2] Run chunk_chapters.py and generate JSON output with chunks + metadata (chapter_id, module, week, heading, position)
- [ ] T136 [US2] Create backend/scripts/generate_embeddings.py to call OpenAI text-embedding-3-small for all chunks
- [ ] T137 [US2] Run generate_embeddings.py and store embeddings + metadata in Qdrant collection "textbook-chunks"
- [ ] T138 [US2] Create backend/scripts/populate_neon.py to insert chunk metadata into Neon chunks table
- [ ] T139 [US2] Run populate_neon.py and verify all chunks have corresponding metadata in Neon
- [ ] T140 [US2] Validate chunk count: 500-1000 chunks total across 6 chapters, storage within Qdrant 1GB limit

### Frontend Chatbot UI

- [ ] T141 [P] [US2] Create src/components/RAGChatbot.tsx React component with input box, send button, message history
- [ ] T142 [US2] Implement chatbot state management (useState for messages, loading state, error state)
- [ ] T143 [US2] Implement /query API call in RAGChatbot.tsx with fetch and error handling
- [ ] T144 [US2] Display chatbot responses with citations as clickable links to chapter sections
- [ ] T145 [US2] Add loading indicator while waiting for response
- [ ] T146 [US2] Create src/css/chatbot.css for chatbot styling (floating sidebar, message bubbles)
- [ ] T147 [US2] Integrate RAGChatbot component into Docusaurus theme (sidebar position, always visible)
- [ ] T148 [US2] Add chatbot toggle button to show/hide chatbot panel

### Select-Text Feature

- [ ] T149 [P] [US2] Create src/components/TextSelector.tsx to detect text selection on page
- [ ] T150 [US2] Add "Ask AI about this" button that appears on text selection
- [ ] T151 [US2] Pass selected text as context to /query API (selected_text parameter)
- [ ] T152 [US2] Integrate TextSelector with RAGChatbot to populate chatbot with selected text + user question

### Testing & Validation

- [ ] T153 [US2] Create backend/tests/test_retrieval.py with 10 test queries (verify top-k chunks relevant)
- [ ] T154 [US2] Create backend/tests/test_api.py with /query endpoint tests (valid request, invalid request, rate limit)
- [ ] T155 [US2] Create 100-query test dataset covering all 6 chapters (mix of Explain/Code/out-of-scope questions)
- [ ] T156 [US2] Run manual evaluation on 100 queries: measure accuracy (>90%), citation correctness, response time (<3s p95)
- [ ] T157 [US2] Test chatbot on 20 out-of-scope queries (e.g., "How to build a drone?") and verify "not covered" responses

**Checkpoint**: At this point, User Story 2 (P2) should be fully functional - chatbot answers questions with citations, select-text feature works, response time <3s

---

## Phase 5: User Story 3 - Instructor Reviews Student Assessment (Priority: P3)

**Goal**: Ensure all chapter assessments have clear problem statements, starter code, and measurable rubrics

**Independent Test**: Review each chapter's assessment section, verify: (1) problem statement is unambiguous, (2) requirements list provided, (3) rubric has measurable criteria with point values

- [ ] T158 [P] [US3] Review Chapter 1 assessment and add starter code template for "Document hardware setup" project
- [ ] T159 [P] [US3] Review Chapter 2 assessment and refine rubric for "Build ROS 2 package" with measurable criteria (e.g., "Node publishes at 10Hz: 5 points")
- [ ] T160 [P] [US3] Review Chapter 3 assessment and add evaluation checklist for "Setup Gazebo environment" project
- [ ] T161 [P] [US3] Review Chapter 4 assessment and add Isaac Sim configuration validation criteria to rubric
- [ ] T162 [P] [US3] Review Chapter 5 assessment and add vision pipeline testing criteria to rubric
- [ ] T163 [P] [US3] Review Chapter 6 capstone rubric and ensure all components have point breakdowns (voice: 20%, navigation: 25%, detection: 25%, manipulation: 30%)
- [ ] T164 [US3] Create docs/appendices/instructor-guide.md with grading guidelines and common student mistakes

**Checkpoint**: At this point, User Story 3 (P3) should be complete - all assessments have clear rubrics and instructor guidelines

---

## Phase 6: User Story 4 - Student Uses Personalization Feature (Priority: P4) [OPTIONAL]

**Goal**: Add user profile modal to customize content based on hardware/OS/language preferences

**Independent Test**: Set profile to "Jetson Orin + Ubuntu + Python", verify Chapter 4 shows Orin-specific commands; switch to "Desktop + Windows + C++", verify content updates

- [ ] T165 [P] [US4] Create src/components/PersonalizationModal.tsx with form fields (name, hardware, OS, language preference)
- [ ] T166 [US4] Implement localStorage persistence for user profile in PersonalizationModal.tsx
- [ ] T167 [US4] Create src/components/ConditionalContent.tsx wrapper component to show/hide content based on profile
- [ ] T168 [US4] Add hardware-specific content variants to Chapter 4 (Jetson Orin vs Desktop sections with ConditionalContent wrapper)
- [ ] T169 [US4] Add OS-specific instructions to Chapter 2 (Ubuntu vs Windows WSL2 setup with ConditionalContent wrapper)
- [ ] T170 [US4] Add language-specific code examples to Chapters 2-5 (Python vs C++ with ConditionalContent wrapper)
- [ ] T171 [US4] Add "Personalize Chapter" button to chapter header that opens PersonalizationModal
- [ ] T172 [US4] Test profile persistence across browser sessions
- [ ] T173 [US4] Add conflict detection (e.g., "Jetson + Windows" → auto-switch to Ubuntu with warning)

**Checkpoint**: Personalization feature complete - content adapts to user preferences stored in localStorage

---

## Phase 7: User Story 5 - Non-English Speaker Reads in Urdu (Priority: P5) [OPTIONAL]

**Goal**: Add Urdu translations for all chapters with RTL layout, language switcher in navbar

**Independent Test**: Switch to Urdu, verify: (1) all prose translated, (2) code blocks unchanged, (3) technical terms remain English, (4) RTL layout correct

- [ ] T174 [P] [US5] Create docs-ur/ directory mirroring docs/ structure (module-01-ros2/, module-02-digital-twin/, etc.)
- [ ] T175 [US5] Configure docusaurus.config.js for i18n with Urdu locale (ur) and RTL direction
- [ ] T176 [US5] Translate Chapter 1 to Urdu in docs-ur/module-01-ros2/week-01-physical-ai-intro.md (prose only, code unchanged)
- [ ] T177 [P] [US5] Translate Chapter 2 to Urdu in docs-ur/module-01-ros2/week-02-ros2-fundamentals.md
- [ ] T178 [P] [US5] Translate Chapter 3 to Urdu in docs-ur/module-02-digital-twin/week-05-digital-twin.md
- [ ] T179 [P] [US5] Translate Chapter 4 to Urdu in docs-ur/module-03-isaac-sim/week-08-isaac-sim.md
- [ ] T180 [P] [US5] Translate Chapter 5 to Urdu in docs-ur/module-04-vla/week-11-vla-systems.md
- [ ] T181 [P] [US5] Translate Chapter 6 to Urdu in docs-ur/capstone/week-13-capstone-project.md
- [ ] T182 [US5] Add Urdu language switcher to navbar (English ↔ اردو toggle)
- [ ] T183 [US5] Create RTL stylesheet in src/css/rtl.css for Urdu layout
- [ ] T184 [US5] Update backend chunking script to support Urdu content (create separate Qdrant collection "textbook-chunks-ur")
- [ ] T185 [US5] Generate embeddings for Urdu chunks and upload to Qdrant
- [ ] T186 [US5] Update backend query router to detect Urdu queries and route to Urdu collection
- [ ] T187 [US5] Test Urdu chatbot with 20 Urdu questions, verify responses in Urdu with correct citations

**Checkpoint**: Urdu translation complete - full textbook available in Urdu with working chatbot

---

## Phase 8: Deployment & Integration Testing

**Purpose**: Deploy to GitHub Pages, deploy backend to Vercel, run end-to-end validation

- [ ] T188 Create .github/workflows/deploy.yml GitHub Actions workflow for automated Docusaurus build and GitHub Pages deployment
- [ ] T189 Configure GitHub repository settings to enable GitHub Pages (source: gh-pages branch)
- [ ] T190 Create backend/vercel.json configuration for Vercel serverless deployment
- [ ] T191 Deploy backend to Vercel free tier and obtain production URL
- [ ] T192 Update frontend RAGChatbot.tsx to use production backend URL (environment variable)
- [ ] T193 Run full Docusaurus build locally and verify build completes in <5 minutes
- [ ] T194 Push to GitHub and trigger automated deployment
- [ ] T195 Verify GitHub Pages deployment successful (access live URL)
- [ ] T196 Run Lighthouse audit on production site (verify performance >90, accessibility >90, SEO >90)
- [ ] T197 Test chatbot on production site with 20 diverse queries
- [ ] T198 Validate rate limiting works on production (test 101 requests in 1 hour)
- [ ] T199 Check all internal links on production site (use link checker tool)
- [ ] T200 Test mobile responsiveness on production (iOS Safari, Android Chrome)
- [ ] T201 Monitor Qdrant usage (verify <1GB storage used)
- [ ] T202 Monitor Neon usage (verify <0.5GB storage used)
- [ ] T203 Configure custom domain if applicable (update DNS, HTTPS)

**Checkpoint**: Production deployment complete - textbook live on GitHub Pages, chatbot backend on Vercel, all metrics met

---

## Phase 9: Polish & Documentation

**Purpose**: Final improvements, documentation, and quality checks

- [ ] T204 [P] Create README.md with project overview, setup instructions, deployment guide, architecture diagram
- [ ] T205 [P] Create backend/README.md with backend-specific setup, API documentation, environment variables
- [ ] T206 [P] Create CONTRIBUTING.md with guidelines for content contributions, code style, PR process
- [ ] T207 Create specs/001-ai-textbook-gen/quickstart.md with 30-minute local setup guide
- [ ] T208 Update constitution.md with any learnings or process improvements from implementation
- [ ] T209 Run final link validation across all chapters
- [ ] T210 Run final code example validation (ensure all code blocks are executable)
- [ ] T211 Review all diagrams for consistency (color scheme, font sizes, labels)
- [ ] T212 Create usage analytics dashboard query in Neon (most queried topics, average response time)
- [ ] T213 Document known limitations and future enhancements in README.md

**Checkpoint**: All documentation complete, project ready for public release

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational frontend complete (T009-T016) - Can start once frontend foundation ready
- **User Story 2 (Phase 4)**: Depends on Foundational backend complete (T017-T029) AND User Story 1 complete (needs chapters for chunking)
- **User Story 3 (Phase 5)**: Depends on User Story 1 complete (needs assessments to review)
- **User Story 4 (Phase 6)**: [OPTIONAL] Depends on User Story 1 complete (needs chapters to personalize)
- **User Story 5 (Phase 7)**: [OPTIONAL] Depends on User Story 1 AND User Story 2 complete (needs English content + chatbot for translation)
- **Deployment (Phase 8)**: Depends on User Story 1 and User Story 2 complete (core functionality)
- **Polish (Phase 9)**: Depends on deployment complete

### User Story Dependencies

- **User Story 1 (P1 - MVP)**: No dependencies on other stories - can implement independently after foundation
- **User Story 2 (P2 - RAG Chatbot)**: Depends on User Story 1 (needs chapter content to chunk and embed)
- **User Story 3 (P3 - Assessment Review)**: Depends on User Story 1 (needs assessments to exist first)
- **User Story 4 (P4 - Personalization)**: [OPTIONAL] Independent after User Story 1
- **User Story 5 (P5 - Urdu Translation)**: [OPTIONAL] Depends on User Story 1 + User Story 2

### Critical Path (Minimum Viable Product)

**MVP = User Story 1 + User Story 2 (P1 + P2)**

1. Phase 1: Setup (T001-T008) → 1 day
2. Phase 2: Foundational (T009-T029) → 2 days
3. Phase 3: User Story 1 - All 6 Chapters (T030-T124) → 10-15 days (content-intensive)
4. Phase 4: User Story 2 - RAG Chatbot (T125-T157) → 5 days
5. Phase 8: Deployment (T188-T203) → 1 day
6. Phase 9: Polish (T204-T213) → 1 day

**Total MVP Timeline: 20-25 days**

### Parallel Opportunities

- **Phase 1 Setup**: All tasks can run in parallel if multiple team members available
- **Phase 2 Foundational**: Frontend tasks (T009-T016) parallel with Backend tasks (T017-T029)
- **Phase 3 User Story 1**: Each chapter can be written in parallel by different authors (T030-T041 || T042-T057 || T058-T073 || T074-T089 || T090-T105 || T106-T116)
- **Phase 4 User Story 2**: Backend implementation (T125-T133) parallel with chunking/embedding (T134-T140) parallel with frontend UI (T141-T152)
- **Phase 6 & 7 Optional Features**: Can be implemented in parallel after User Story 1 complete

---

## Implementation Strategy

### MVP First (Fastest Path to Value)

1. Complete Phase 1: Setup (T001-T008)
2. Complete Phase 2: Foundational (T009-T029)
3. Complete Phase 3: User Story 1 - Focus on Chapters 1-2 first for quick validation (T030-T057)
4. **VALIDATE**: Test Chapters 1-2 render correctly, diagrams work, code examples highlighted
5. Complete remaining Chapters 3-6 (T058-T124)
6. Complete Phase 4: User Story 2 - RAG Chatbot (T125-T157)
7. **VALIDATE**: Test chatbot answers questions from all chapters accurately
8. Deploy MVP (Phase 8: T188-T203)
9. Add optional features (Phase 5-7) if time/budget permits

### Incremental Delivery (Staged Releases)

- **Release 1 (MVP)**: Setup + Foundation + User Story 1 (Chapters 1-6) + User Story 2 (Chatbot) → Full textbook with AI assistant
- **Release 2**: Add User Story 3 (Instructor tools) → Educational institution ready
- **Release 3**: Add User Story 4 (Personalization) → Enhanced UX
- **Release 4**: Add User Story 5 (Urdu translation) → Inclusivity for Urdu speakers

### Parallel Team Strategy

**Team of 3 developers:**

1. **Week 1**: All → Phase 1 + Phase 2 (Setup + Foundation)
2. **Week 2-3**:
   - Dev A: Chapters 1-2 (T030-T057)
   - Dev B: Chapters 3-4 (T058-T089)
   - Dev C: Chapters 5-6 + Appendices (T090-T119)
3. **Week 4**: All → Polish chapters + Frontend testing (T120-T124)
4. **Week 5**:
   - Dev A: Backend RAG implementation (T125-T133)
   - Dev B: Chunking + Embedding (T134-T140)
   - Dev C: Chatbot UI (T141-T152)
5. **Week 6**: All → Testing, deployment, polish (T153-T213)

---

## Success Metrics Validation

After implementation, validate against success criteria from spec.md:

### Content Quality (SC-001 to SC-005)
- [ ] All 6 chapters complete with 100% mandatory sections ✓
- [ ] 20+ diagrams total (3+ per chapter) ✓
- [ ] 50+ tested code examples ✓
- [ ] 5+ MCQs and 1 assignment per chapter ✓
- [ ] Capstone chapter has complete end-to-end architecture ✓

### RAG Chatbot Performance (SC-006 to SC-010)
- [ ] Chatbot accuracy >90% on 100-query test set ✓
- [ ] Response time <3s p95 ✓
- [ ] Zero hallucinations in test set ✓
- [ ] Out-of-scope queries handled correctly ✓
- [ ] Query routing accuracy >95% ✓

### User Experience (SC-011 to SC-015)
- [ ] Lighthouse performance >90 ✓
- [ ] First Contentful Paint <1.5s on 3G ✓
- [ ] Responsive 320px-2560px ✓
- [ ] Navigation to any chapter in <5 clicks ✓
- [ ] Dark mode toggle works without flicker ✓

### Deployment & Reliability (SC-016 to SC-020)
- [ ] GitHub Pages deployment successful ✓
- [ ] Build time <5 minutes ✓
- [ ] Zero 404 errors (link validation) ✓
- [ ] Chatbot uptime >99% (30-day rolling) ✓
- [ ] Within free-tier limits (Qdrant <1GB, Neon <0.5GB) ✓

---

## Notes

- **[P] tasks**: Different files, no dependencies, can run in parallel
- **[Story] labels**: Map tasks to user stories for traceability
- **Content-heavy**: Phase 3 (User Story 1) is most time-intensive - writing 6 comprehensive chapters with diagrams, code, assessments
- **Validation checkpoints**: Test each chapter independently before moving to next
- **Optional features**: Phases 6-7 (Personalization, Urdu) can be deferred for MVP
- **Free-tier constraints**: Monitor Qdrant (1GB) and Neon (0.5GB) usage throughout implementation
- **ADR suggestions**: Create ADRs for key decisions (Docusaurus choice, Qdrant selection, chunking strategy) per constitution guidelines

**Ready to proceed with implementation following task sequence!**
