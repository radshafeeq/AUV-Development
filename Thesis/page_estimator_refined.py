import re

def estimate_pages(filepath):
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Split by double dollar to find math blocks
    math_blocks_raw = re.findall(r'\$\$(.*?)\$\$', content, flags=re.DOTALL)
    
    inline_math = 0
    multiline_math = 0
    
    for block in math_blocks_raw:
        if '\n' in block or '\\begin' in block:
            multiline_math += 1
        else:
            inline_math += 1
            
    # Text words
    text_only = re.sub(r'\$\$.*?\$\$', 'X', content, flags=re.DOTALL)
    words = len(text_only.split())
    
    # 1 page = 250 words
    # 1 multiline math = ~1/5 page (approx 4-5 lines)
    # Inline math is counted inside the text flow
    
    word_pages = words / 250.0
    math_pages = multiline_math * 0.15
    
    total_pages = word_pages + math_pages
    return words, inline_math, multiline_math, total_pages

files = [
    "BAGIAN_AWAL_PROPOSAL.md",
    "BAB_1_PENDAHULUAN.md",
    "BAB_2_LANDASAN_TEORI.md"
]

total = 0
for file in files:
    w, im, mm, p = estimate_pages(f"/home/radhi/Documents/AUV Development/Thesis/{file}")
    print(f"{file}:")
    print(f"  - Words: {w}")
    print(f"  - Inline Math: {im}")
    print(f"  - Multiline Math: {mm}")
    print(f"  => Estimated Pages: ~{p:.1f}")
    total += p

print(f"\nTotal Estimated Pages: ~{total:.1f}")
