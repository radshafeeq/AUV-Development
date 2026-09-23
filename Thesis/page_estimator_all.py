import re

def estimate_pages(filepath):
    with open(filepath, 'r') as f:
        content = f.read()
    
    math_blocks_raw = re.findall(r'\$\$(.*?)\$\$', content, flags=re.DOTALL)
    
    inline_math = 0
    multiline_math = 0
    
    for block in math_blocks_raw:
        if '\n' in block or '\\begin' in block:
            multiline_math += 1
        else:
            inline_math += 1
            
    text_only = re.sub(r'\$\$.*?\$\$', 'X', content, flags=re.DOTALL)
    words = len(text_only.split())
    
    word_pages = words / 250.0
    math_pages = multiline_math * 0.15
    
    total_pages = word_pages + math_pages
    return words, inline_math, multiline_math, total_pages

files = [
    "BAGIAN_AWAL_PROPOSAL.md",
    "BAB_1_PENDAHULUAN.md",
    "BAB_2_LANDASAN_TEORI.md",
    "BAB_3_METODOLOGI_PENELITIAN.md"
]

total = 0
print("--- SUMMARY OF ESTIMATED THESIS PAGES ---")
for file in files:
    w, im, mm, p = estimate_pages(f"/home/radhi/Documents/AUV Development/Thesis/{file}")
    print(f"{file}:")
    print(f"  - Words: {w}")
    print(f"  - Inline Math ($$x$$): {im}")
    print(f"  - Multiline Math Equations: {mm}")
    print(f"  => Estimated Pages: ~{p:.1f}")
    total += p

print(f"\n==========================================")
print(f"TOTAL ESTIMATED THESIS PROPOSAL PAGES: ~{total:.1f} Pages")
print(f"==========================================")
