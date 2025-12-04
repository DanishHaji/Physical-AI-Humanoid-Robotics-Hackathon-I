"""
Text Chunking Module
Semantic chunking by heading with configurable overlap
Optimized for Markdown textbook chapters
"""

import re
from typing import List, Dict, Optional
from dataclasses import dataclass
from .embeddings import count_tokens


@dataclass
class Chunk:
    """Represents a text chunk with metadata"""
    content: str
    heading: str
    position: int
    token_count: int
    metadata: Dict


def extract_headings(markdown_text: str) -> List[Dict]:
    """
    Extract all headings from Markdown text with their positions

    Args:
        markdown_text: Markdown content

    Returns:
        List of dicts containing heading info (level, text, start_pos, end_pos)
    """
    heading_pattern = r'^(#{1,6})\s+(.+)$'
    headings = []

    lines = markdown_text.split('\n')
    char_position = 0

    for line_num, line in enumerate(lines):
        match = re.match(heading_pattern, line, re.MULTILINE)
        if match:
            level = len(match.group(1))  # Number of # symbols
            text = match.group(2).strip()

            headings.append({
                'level': level,
                'text': text,
                'line_num': line_num,
                'start_pos': char_position,
                'heading_line': line
            })

        char_position += len(line) + 1  # +1 for newline

    return headings


def chunk_by_heading(
    markdown_text: str,
    min_tokens: int = 512,
    max_tokens: int = 1024,
    overlap_tokens: int = 100
) -> List[Chunk]:
    """
    Chunk Markdown text by headings with semantic boundaries

    Strategy:
    1. Split at ## (h2) headings (major sections)
    2. If section > max_tokens, recursively split at ### (h3) headings
    3. If still > max_tokens, split at paragraph boundaries
    4. Add overlap_tokens from end of chunk N to beginning of chunk N+1

    Args:
        markdown_text: Markdown content to chunk
        min_tokens: Minimum tokens per chunk (default: 512)
        max_tokens: Maximum tokens per chunk (default: 1024)
        overlap_tokens: Number of tokens to overlap between chunks (default: 100)

    Returns:
        List of Chunk objects
    """
    headings = extract_headings(markdown_text)

    if not headings:
        # No headings found, treat entire text as one chunk
        return [create_single_chunk(markdown_text, "Introduction", 0)]

    chunks = []
    position = 0

    # Find all h2 (##) headings as primary splitting points
    h2_headings = [h for h in headings if h['level'] == 2]

    if not h2_headings:
        # If no h2 headings, use h1 or first available heading level
        h2_headings = [h for h in headings if h['level'] == min(hd['level'] for hd in headings)]

    for i, heading in enumerate(h2_headings):
        # Determine section boundaries
        start_pos = heading['start_pos']
        end_pos = h2_headings[i + 1]['start_pos'] if i + 1 < len(h2_headings) else len(markdown_text)

        section_text = markdown_text[start_pos:end_pos].strip()
        section_heading = heading['text']

        # Count tokens in section
        section_tokens = count_tokens(section_text)

        if section_tokens <= max_tokens:
            # Section fits in one chunk
            if section_tokens >= min_tokens:
                chunks.append(Chunk(
                    content=section_text,
                    heading=section_heading,
                    position=position,
                    token_count=section_tokens,
                    metadata={'heading_level': 2}
                ))
                position += 1
            else:
                # Section too small, merge with previous chunk if possible
                if chunks and chunks[-1].token_count + section_tokens <= max_tokens:
                    # Merge with previous chunk
                    chunks[-1].content += "\n\n" + section_text
                    chunks[-1].token_count = count_tokens(chunks[-1].content)
                else:
                    # Create chunk anyway (even if small)
                    chunks.append(Chunk(
                        content=section_text,
                        heading=section_heading,
                        position=position,
                        token_count=section_tokens,
                        metadata={'heading_level': 2}
                    ))
                    position += 1
        else:
            # Section too large, split further
            sub_chunks = split_large_section(
                section_text,
                section_heading,
                max_tokens,
                overlap_tokens
            )

            for sub_chunk in sub_chunks:
                sub_chunk.position = position
                chunks.append(sub_chunk)
                position += 1

    # Add overlap between consecutive chunks
    chunks_with_overlap = add_overlap(chunks, overlap_tokens)

    return chunks_with_overlap


def split_large_section(
    section_text: str,
    section_heading: str,
    max_tokens: int,
    overlap_tokens: int
) -> List[Chunk]:
    """
    Split a large section that exceeds max_tokens

    Strategy:
    1. Try splitting at h3 (###) headings
    2. If still too large, split at paragraph boundaries
    3. Ensure each chunk is ≤ max_tokens

    Args:
        section_text: Section content
        section_heading: Section heading text
        max_tokens: Maximum tokens per chunk
        overlap_tokens: Overlap tokens

    Returns:
        List of Chunk objects
    """
    # Extract h3 (###) subheadings within this section
    h3_pattern = r'^###\s+(.+)$'
    subheadings = []

    lines = section_text.split('\n')
    char_pos = 0

    for line_num, line in enumerate(lines):
        if re.match(h3_pattern, line, re.MULTILINE):
            subheadings.append({
                'text': line.replace('###', '').strip(),
                'line_num': line_num,
                'start_pos': char_pos
            })
        char_pos += len(line) + 1

    if len(subheadings) > 1:
        # Split at h3 headings
        chunks = []
        for i, subheading in enumerate(subheadings):
            start_pos = subheading['start_pos']
            end_pos = subheadings[i + 1]['start_pos'] if i + 1 < len(subheadings) else len(section_text)

            sub_text = section_text[start_pos:end_pos].strip()
            sub_tokens = count_tokens(sub_text)

            if sub_tokens <= max_tokens:
                chunks.append(Chunk(
                    content=sub_text,
                    heading=f"{section_heading} > {subheading['text']}",
                    position=0,  # Will be set by parent function
                    token_count=sub_tokens,
                    metadata={'heading_level': 3}
                ))
            else:
                # Still too large, split at paragraphs
                para_chunks = split_by_paragraphs(
                    sub_text,
                    f"{section_heading} > {subheading['text']}",
                    max_tokens
                )
                chunks.extend(para_chunks)

        return chunks
    else:
        # No h3 headings, split at paragraph boundaries
        return split_by_paragraphs(section_text, section_heading, max_tokens)


def split_by_paragraphs(
    text: str,
    heading: str,
    max_tokens: int
) -> List[Chunk]:
    """
    Split text at paragraph boundaries to respect max_tokens

    Args:
        text: Text to split
        heading: Heading for these chunks
        max_tokens: Maximum tokens per chunk

    Returns:
        List of Chunk objects
    """
    # Split by double newlines (paragraphs)
    paragraphs = text.split('\n\n')

    chunks = []
    current_chunk = ""
    current_tokens = 0

    for para in paragraphs:
        para_tokens = count_tokens(para)

        if current_tokens + para_tokens <= max_tokens:
            # Add paragraph to current chunk
            current_chunk += para + "\n\n"
            current_tokens += para_tokens
        else:
            # Save current chunk and start new one
            if current_chunk.strip():
                chunks.append(Chunk(
                    content=current_chunk.strip(),
                    heading=heading,
                    position=0,  # Will be set by parent function
                    token_count=current_tokens,
                    metadata={'heading_level': 2, 'split_type': 'paragraph'}
                ))

            # Start new chunk with current paragraph
            current_chunk = para + "\n\n"
            current_tokens = para_tokens

    # Add final chunk
    if current_chunk.strip():
        chunks.append(Chunk(
            content=current_chunk.strip(),
            heading=heading,
            position=0,
            token_count=current_tokens,
            metadata={'heading_level': 2, 'split_type': 'paragraph'}
        ))

    return chunks


def add_overlap(chunks: List[Chunk], overlap_tokens: int) -> List[Chunk]:
    """
    Add overlap between consecutive chunks

    Takes last overlap_tokens from chunk N and prepends to chunk N+1

    Args:
        chunks: List of chunks
        overlap_tokens: Number of tokens to overlap

    Returns:
        List of chunks with overlap added
    """
    if len(chunks) <= 1 or overlap_tokens == 0:
        return chunks

    overlapped_chunks = [chunks[0]]  # First chunk has no prefix overlap

    for i in range(1, len(chunks)):
        prev_chunk = chunks[i - 1]
        curr_chunk = chunks[i]

        # Extract last overlap_tokens from previous chunk
        prev_tokens = count_tokens(prev_chunk.content)

        if prev_tokens > overlap_tokens:
            # Get last sentences/words from previous chunk
            overlap_text = get_last_n_tokens(prev_chunk.content, overlap_tokens)

            # Prepend overlap to current chunk
            new_content = overlap_text + "\n\n" + curr_chunk.content
            new_token_count = count_tokens(new_content)

            overlapped_chunks.append(Chunk(
                content=new_content,
                heading=curr_chunk.heading,
                position=curr_chunk.position,
                token_count=new_token_count,
                metadata={**curr_chunk.metadata, 'has_overlap': True}
            ))
        else:
            # Previous chunk too small for overlap, keep current chunk as is
            overlapped_chunks.append(curr_chunk)

    return overlapped_chunks


def get_last_n_tokens(text: str, n: int) -> str:
    """
    Extract last n tokens from text (approximately, using word-based splitting)

    Args:
        text: Input text
        n: Number of tokens

    Returns:
        str: Last n tokens (approximately)
    """
    from .embeddings import encoding

    tokens = encoding.encode(text)
    if len(tokens) <= n:
        return text

    # Get last n tokens
    last_tokens = tokens[-n:]
    return encoding.decode(last_tokens)


def create_single_chunk(text: str, heading: str, position: int) -> Chunk:
    """
    Create a single chunk from text

    Args:
        text: Text content
        heading: Heading text
        position: Chunk position

    Returns:
        Chunk object
    """
    return Chunk(
        content=text.strip(),
        heading=heading,
        position=position,
        token_count=count_tokens(text),
        metadata={'heading_level': 1, 'single_chunk': True}
    )


def chunk_markdown_file(
    file_path: str,
    min_tokens: int = 512,
    max_tokens: int = 1024,
    overlap_tokens: int = 100
) -> List[Chunk]:
    """
    Read and chunk a Markdown file

    Args:
        file_path: Path to Markdown file
        min_tokens: Minimum tokens per chunk
        max_tokens: Maximum tokens per chunk
        overlap_tokens: Overlap tokens

    Returns:
        List of Chunk objects
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    return chunk_by_heading(content, min_tokens, max_tokens, overlap_tokens)


def validate_chunks(chunks: List[Chunk], min_tokens: int, max_tokens: int) -> Dict:
    """
    Validate that chunks meet token constraints

    Args:
        chunks: List of chunks to validate
        min_tokens: Minimum tokens per chunk
        max_tokens: Maximum tokens per chunk

    Returns:
        Dict with validation results
    """
    total_chunks = len(chunks)
    valid_chunks = 0
    too_small = 0
    too_large = 0

    for chunk in chunks:
        if min_tokens <= chunk.token_count <= max_tokens:
            valid_chunks += 1
        elif chunk.token_count < min_tokens:
            too_small += 1
        else:
            too_large += 1

    return {
        'total_chunks': total_chunks,
        'valid_chunks': valid_chunks,
        'too_small': too_small,
        'too_large': too_large,
        'valid_percentage': round((valid_chunks / total_chunks) * 100, 2) if total_chunks > 0 else 0
    }
