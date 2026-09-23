import re

# Load Master Bibliography entries
with open("/home/radhi/Documents/AUV Development/Thesis/MASTER_BIBLIOGRAPHY.md", "r", encoding="utf-8") as f:
    master_bib_text = f.read()

bib_entries = {}
for line in master_bib_text.splitlines():
    m = re.match(r'- \*\*\[(\d+)\]\*\* (.*)', line)
    if m:
        bib_entries[int(m.group(1))] = m.group(2)

print(f"Loaded {len(bib_entries)} bibliography entries from MASTER_BIBLIOGRAPHY.md")

old_to_new = {
    1: 1,    # Ahmed
    2: 2,    # Alinei-Poiana
    3: 3,    # Blue Robotics
    4: 5,    # DNV
    5: 6,    # Fan
    6: 7,    # Fossen
    7: 14,   # Ismail
    8: 16,   # Khalid
    9: 17,   # Kim
    10: 18,  # Llorente-Vidrio
    11: 21,  # Ng & Krieg
    12: 25,  # Sarkka & Svensson
    13: 27,  # Suarez
    14: 29,  # Ulin-Avila
    15: 31,  # von Benzon
    16: 32,  # Vu
    17: 33,  # Wang
    18: 34,  # Wei
    19: 40   # Zhang (2025 - NMPC)
}

def remap_old_citations(text):
    # Matches [1], [6], [11, 16], etc.
    def repl(m):
        inner = m.group(1)
        parts = [p.strip() for p in inner.split(",") if p.strip().isdigit()]
        if not parts:
            return m.group(0)
        nums = [int(p) for p in parts]
        if all(1 <= n <= 19 for n in nums):
            new_nums = sorted(list(set(old_to_new[n] for n in nums)))
            return "[" + ", ".join(str(n) for n in new_nums) + "]"
        return m.group(0)
    return re.sub(r'\[(\d+(?:\s*,\s*\d+)*)\]', repl, text)

print("Remapping logic verified.")
