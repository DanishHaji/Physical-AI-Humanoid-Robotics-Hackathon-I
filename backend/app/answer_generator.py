"""
Answer Generation Module
Generate answers using OpenAI GPT-4o-mini with retrieved context
"""

from openai import AsyncOpenAI
from typing import List, Dict
from .config import settings

# Initialize OpenAI client
client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)


async def generate_answer(
    question: str,
    context_chunks: List[Dict],
    mode: str = "explain",
    selected_text: str = None
) -> str:
    """
    Generate answer using OpenAI GPT-4o-mini with retrieved context

    Args:
        question: User's question
        context_chunks: List of retrieved chunks with content and metadata
        mode: Query mode (explain, code, urdu, exam)
        selected_text: Optional selected text for contextual queries

    Returns:
        str: Generated answer
    """
    try:
        # Build context string from chunks
        context = build_context(context_chunks)

        # Build system prompt based on mode
        system_prompt = build_system_prompt(mode)

        # Build user message
        user_message = build_user_message(question, context, selected_text)

        # Call OpenAI API
        response = await client.chat.completions.create(
            model=settings.OPENAI_CHAT_MODEL,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message}
            ],
            max_tokens=settings.OPENAI_MAX_TOKENS,
            temperature=settings.OPENAI_TEMPERATURE
        )

        # Extract answer
        answer = response.choices[0].message.content.strip()

        print(f"✅ Generated answer ({len(answer)} chars) using {response.usage.total_tokens} tokens")
        return answer

    except Exception as e:
        print(f"❌ Error generating answer: {str(e)}")
        raise


def build_context(chunks: List[Dict]) -> str:
    """
    Build context string from retrieved chunks

    Args:
        chunks: List of chunk dicts

    Returns:
        str: Formatted context string
    """
    if not chunks:
        return "No relevant content found in the textbook."

    context_parts = []

    for i, chunk in enumerate(chunks, 1):
        chapter_title = chunk.get('chapter_title', 'Unknown Chapter')
        heading = chunk.get('heading', 'Unknown Section')
        content = chunk.get('content', '').strip()

        context_parts.append(
            f"[Source {i}: {chapter_title} - {heading}]\n{content}\n"
        )

    return "\n".join(context_parts)


def build_system_prompt(mode: str) -> str:
    """
    Build system prompt based on query mode

    Args:
        mode: Query mode (explain, code, urdu, exam)

    Returns:
        str: System prompt
    """
    base_prompt = """You are an AI teaching assistant for the "Physical AI & Humanoid Robotics" textbook.

CRITICAL RULES:
1. Answer ONLY using information from the provided textbook content (Sources)
2. If the answer is not in the provided sources, say "This topic is not covered in the provided textbook sections. Please check the table of contents for related chapters."
3. Always cite sources using format: (Source 1: Chapter Name - Section)
4. Be concise but technically accurate
5. Do not hallucinate or add information not present in the sources"""

    mode_specific = {
        "explain": """
Focus on conceptual explanations with clear, pedagogical language.
Break down complex topics step-by-step.
Use analogies when helpful.
""",
        "code": """
Focus on code examples and implementation details.
Provide working code snippets when available in sources.
Explain what each code section does.
Include any setup instructions or prerequisites mentioned in sources.
""",
        "urdu": """
Respond in Urdu (اردو) language.
Use technical terms in English but explain concepts in Urdu.
Maintain the same citation format.
""",
        "exam": """
Focus on assessment and evaluation.
Provide concise, exam-style answers.
Highlight key points that would be tested.
Include relevant formulas or definitions.
"""
    }

    return base_prompt + mode_specific.get(mode, mode_specific["explain"])


def build_user_message(question: str, context: str, selected_text: str = None) -> str:
    """
    Build user message with question and context

    Args:
        question: User's question
        context: Retrieved textbook content
        selected_text: Optional selected text

    Returns:
        str: Formatted user message
    """
    if selected_text:
        return f"""TEXTBOOK CONTENT:
{context}

USER SELECTED TEXT:
{selected_text}

QUESTION: {question}

Please answer based on the textbook content and the selected text."""
    else:
        return f"""TEXTBOOK CONTENT:
{context}

QUESTION: {question}

Please answer based only on the textbook content above."""


async def generate_no_context_response(question: str, mode: str = "explain") -> str:
    """
    Generate response when no relevant context is found

    Args:
        question: User's question
        mode: Query mode

    Returns:
        str: Helpful fallback response
    """
    fallback_responses = {
        "explain": f"I couldn't find relevant information in the textbook for your question: '{question}'. This textbook covers Physical AI and Humanoid Robotics topics including ROS 2, Digital Twin simulation, NVIDIA Isaac, and Vision-Language-Action systems. Please try rephrasing your question or check the table of contents for available chapters.",

        "code": f"I couldn't find code examples for '{question}' in the available textbook sections. The textbook includes code examples for ROS 2 nodes, URDF models, Gazebo simulation, Isaac Sim, and VLA systems. Please check specific chapter sections for implementation details.",

        "urdu": f"آپ کے سوال '{question}' کے لیے کتاب میں متعلقہ معلومات نہیں ملیں۔ یہ کتاب Physical AI اور Humanoid Robotics کے موضوعات پر مشتمل ہے جیسے ROS 2، Digital Twin simulation، NVIDIA Isaac، اور Vision-Language-Action systems۔ براہ کرم اپنا سوال دوبارہ لکھیں یا فہرست دیکھیں۔",

        "exam": f"Question not covered in the current textbook sections. Topics covered include: ROS 2 fundamentals, Digital Twin simulation, NVIDIA Isaac platform, and VLA systems. Please refer to chapter assessments for exam-style questions."
    }

    return fallback_responses.get(mode, fallback_responses["explain"])


async def test_answer_generation() -> bool:
    """
    Test answer generation with dummy context

    Returns:
        bool: True if successful, False otherwise
    """
    try:
        test_question = "What is ROS 2?"
        test_context = [{
            'content': "ROS 2 (Robot Operating System 2) is an open-source framework for building robot applications.",
            'chapter_title': "ROS 2 Fundamentals",
            'heading': "Introduction to ROS 2"
        }]

        answer = await generate_answer(test_question, test_context, mode="explain")

        if len(answer) > 10:
            print(f"✅ Answer generation test successful")
            return True
        else:
            print(f"❌ Answer generation test failed: answer too short")
            return False

    except Exception as e:
        print(f"❌ Answer generation test failed: {str(e)}")
        return False


def estimate_answer_cost(
    prompt_tokens: int,
    completion_tokens: int
) -> float:
    """
    Estimate cost for answer generation using gpt-4o-mini

    Args:
        prompt_tokens: Number of input tokens
        completion_tokens: Number of output tokens

    Returns:
        float: Estimated cost in USD
    """
    # gpt-4o-mini pricing: $0.15/1M input tokens, $0.60/1M output tokens
    input_price_per_million = 0.15
    output_price_per_million = 0.60

    input_cost = (prompt_tokens / 1_000_000) * input_price_per_million
    output_cost = (completion_tokens / 1_000_000) * output_price_per_million

    total_cost = input_cost + output_cost
    return round(total_cost, 6)
