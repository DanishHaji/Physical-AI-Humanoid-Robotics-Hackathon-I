"""
Groq API Client for LLM inference (Cloud Free-Tier)
Provides chat completion using Groq's llama-3.1-8b-instant model
Sign up: https://console.groq.com (30 requests/min free)
"""

from groq import Groq
from typing import List, Dict, Optional
from ..config import settings


class GroqClient:
    """
    Wrapper for Groq API client with rate limiting and error handling
    """

    def __init__(self, api_key: Optional[str] = None):
        """
        Initialize Groq client

        Args:
            api_key: Groq API key (defaults to settings.GROQ_API_KEY)

        Raises:
            ValueError: If API key is not provided
        """
        self.api_key = api_key or settings.GROQ_API_KEY

        if not self.api_key:
            raise ValueError(
                "Groq API key not found. "
                "Sign up at https://console.groq.com and set GROQ_API_KEY in .env"
            )

        # Initialize Groq client
        self.client = Groq(api_key=self.api_key)
        self.model = settings.GROQ_MODEL
        self.temperature = settings.GROQ_TEMPERATURE
        self.max_tokens = settings.GROQ_MAX_TOKENS

        print(f"[OK] Groq client initialized (model: {self.model})")

    def chat_completion(
        self,
        messages: List[Dict[str, str]],
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        stream: bool = False
    ) -> str:
        """
        Generate chat completion using Groq API

        Args:
            messages: List of message dicts with 'role' and 'content' keys
                     Example: [{"role": "system", "content": "..."}, {"role": "user", "content": "..."}]
            temperature: Override default temperature (0.0-2.0)
            max_tokens: Override default max tokens
            stream: Whether to stream the response (default: False)

        Returns:
            str: Generated text response

        Raises:
            Exception: If API call fails
        """
        try:
            # Use defaults if not specified
            temp = temperature if temperature is not None else self.temperature
            tokens = max_tokens if max_tokens is not None else self.max_tokens

            # Call Groq API
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=temp,
                max_tokens=tokens,
                stream=stream
            )

            # Extract response text
            if stream:
                # For streaming, return the full response (caller should handle streaming)
                return response
            else:
                answer = response.choices[0].message.content.strip()
                print(f"[OK] Groq response generated ({len(answer)} chars)")
                return answer

        except Exception as e:
            print(f"[ERROR] Groq API error: {str(e)}")

            # Provide helpful error messages
            if "api_key" in str(e).lower() or "authentication" in str(e).lower():
                raise RuntimeError(
                    "Groq API authentication failed. "
                    "Check your GROQ_API_KEY in .env or get a new one from https://console.groq.com"
                )
            elif "rate_limit" in str(e).lower() or "429" in str(e):
                raise RuntimeError(
                    "Groq rate limit exceeded (30 req/min). "
                    "Please wait a moment and try again."
                )
            else:
                raise RuntimeError(f"Groq API error: {str(e)}")

    def test_connection(self) -> bool:
        """
        Test Groq API connection with a simple request

        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            test_messages = [
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": "Say 'OK' if you can read this."}
            ]

            response = self.chat_completion(
                messages=test_messages,
                max_tokens=10
            )

            if len(response) > 0:
                print(f"[OK] Groq connection test successful")
                return True
            else:
                print(f"[ERROR] Groq connection test failed: empty response")
                return False

        except Exception as e:
            print(f"[ERROR] Groq connection test failed: {str(e)}")
            return False

    def get_model_info(self) -> Dict:
        """
        Get information about the Groq model configuration

        Returns:
            dict: Model metadata
        """
        return {
            "provider": "Groq",
            "model_name": self.model,
            "temperature": self.temperature,
            "max_tokens": self.max_tokens,
            "cost": "FREE (30 requests/min)",
            "base_url": settings.GROQ_BASE_URL,
            "api_key_configured": bool(self.api_key)
        }


# Global instance (initialized when needed)
_groq_client: Optional[GroqClient] = None


def get_groq_client() -> GroqClient:
    """
    Get or create global Groq client instance

    Returns:
        GroqClient instance

    Raises:
        ValueError: If API key is not configured
    """
    global _groq_client

    if _groq_client is None:
        _groq_client = GroqClient()

    return _groq_client


def generate_answer_with_groq(
    question: str,
    context: str,
    mode: str = "explain",
    selected_text: Optional[str] = None
) -> str:
    """
    High-level function to generate answers using Groq

    Args:
        question: User's question
        context: Retrieved textbook content
        mode: Query mode (explain, code, urdu, exam)
        selected_text: Optional selected text for context

    Returns:
        str: Generated answer

    Raises:
        RuntimeError: If Groq API fails
    """
    client = get_groq_client()

    # Build system prompt based on mode
    system_prompts = {
        "explain": """You are an AI teaching assistant for the "Physical AI & Humanoid Robotics" textbook.

CRITICAL RULES:
1. Answer ONLY using information from the provided textbook content (Sources)
2. If the answer is not in the provided sources, say "This topic is not covered in the provided textbook sections."
3. Always cite sources using format: (Source 1: Chapter Name - Section)
4. Be concise but technically accurate
5. Do not hallucinate or add information not present in the sources

Focus on conceptual explanations with clear, pedagogical language.""",

        "code": """You are an AI teaching assistant for the "Physical AI & Humanoid Robotics" textbook.

CRITICAL RULES:
1. Answer ONLY using code examples from the provided textbook content
2. If code is not in the sources, say "No code examples found for this topic."
3. Cite sources for all code snippets
4. Explain what each code section does

Focus on implementation details and working code examples.""",

        "urdu": """You are an AI teaching assistant for the "Physical AI & Humanoid Robotics" textbook.

CRITICAL RULES:
1. Respond in Urdu (اردو) language
2. Answer ONLY using information from the provided sources
3. Use technical terms in English but explain concepts in Urdu
4. Maintain citation format

اردو میں جواب دیں لیکن تکنیکی اصطلاحات انگریزی میں استعمال کریں۔""",

        "exam": """You are an AI teaching assistant for the "Physical AI & Humanoid Robotics" textbook.

CRITICAL RULES:
1. Answer ONLY from provided sources
2. Provide concise, exam-style answers
3. Highlight key points that would be tested
4. Include relevant formulas or definitions

Focus on assessment and evaluation."""
    }

    system_prompt = system_prompts.get(mode, system_prompts["explain"])

    # Build user message
    if selected_text:
        user_message = f"""TEXTBOOK CONTENT:
{context}

USER SELECTED TEXT:
{selected_text}

QUESTION: {question}

Please answer based on the textbook content and the selected text."""
    else:
        user_message = f"""TEXTBOOK CONTENT:
{context}

QUESTION: {question}

Please answer based only on the textbook content above."""

    # Generate response
    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_message}
    ]

    return client.chat_completion(messages=messages)


# Performance comparison (for reference)
"""
Groq vs OpenAI Comparison:
┌─────────────────────┬─────────────────┬──────────────────────┐
│ Feature             │ OpenAI          │ Groq                 │
├─────────────────────┼─────────────────┼──────────────────────┤
│ Model               │ gpt-4o-mini     │ llama-3.1-8b-instant │
│ Parameters          │ Unknown         │ 8 billion            │
│ Cost                │ $0.15/1M in     │ FREE                 │
│                     │ $0.60/1M out    │                      │
│ Speed               │ ~500ms          │ ~400ms (faster!)     │
│ Rate Limits         │ 10K TPM         │ 30 req/min           │
│ Quality             │ Very High       │ Good                 │
│ Context Window      │ 128K tokens     │ 8K tokens            │
└─────────────────────┴─────────────────┴──────────────────────┘

For textbook RAG:
- Groq is FREE and fast (LPU architecture)
- llama-3.1-8b-instant is sufficient for educational Q&A
- 30 req/min is enough for small deployments
- Can upgrade to paid tier for production (60 req/min)
"""
