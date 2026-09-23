import os

with open("/home/radhi/Documents/AUV_GitHub_Upload/SIMULATION_REQUIREMENTS.md") as f:
    text = f.read()
print("SIMULATION REQUIREMENTS (first 1000 chars):")
print(text[:1000])

print("\n--- Listing key files in AUV_GitHub_Upload ---")
for fname in os.listdir("/home/radhi/Documents/AUV_GitHub_Upload"):
    if fname.endswith(".sh") or fname.endswith(".py") or fname.endswith(".md"):
        print(fname)
