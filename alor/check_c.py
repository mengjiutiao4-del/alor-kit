"""Sanity-check a driver .c file before it goes to CI.

Two checks, both born from real losses:
  * brace/paren balance and leftover anchors from a patch script,
  * a string literal broken by a real newline instead of \\n - the recurring
    python-patch mistake that cost a 20-minute build round on 2026-10-30.
Exits non-zero so CI can gate on it.
"""
import sys

path = sys.argv[1]
s = open(path, encoding='utf-8', errors='surrogateescape').read()
lines = s.split('\n')

bad_strings = []
for n, line in enumerate(lines, 1):
    t = line.strip()
    if t.startswith('*') or t.startswith('//') or t.startswith('/*'):
        continue
    quotes = 0
    i = 0
    while i < len(line):
        c = line[i]
        if c == '\\':
            i += 2
            continue
        if c == '"':
            quotes += 1
        i += 1
    if quotes % 2:
        bad_strings.append((n, t[:70]))

d = p = 0
ln = 1
neg = None
i = 0
while i < len(s):
    c = s[i]
    if c == '\n':
        ln += 1
        i += 1
        continue
    if c == '/' and i + 1 < len(s) and s[i + 1] == '/':
        j = s.find('\n', i)
        i = j if j >= 0 else len(s)
        continue
    if c == '/' and i + 1 < len(s) and s[i + 1] == '*':
        j = s.find('*/', i)
        i = j + 2 if j >= 0 else len(s)
        continue
    if c in '"\'':
        q = c
        i += 1
        while i < len(s) and s[i] != q:
            if s[i] == '\\':
                i += 1
            elif s[i] == '\n':
                bad_strings.append((s[:i].count('\n') + 1, 'unterminated literal'))
                break
            i += 1
        i += 1
        continue
    if c == '{':
        d += 1
    elif c == '}':
        d -= 1
        if d < 0 and neg is None:
            neg = ln
    elif c == '(':
        p += 1
    elif c == ')':
        p -= 1
    i += 1

print('brace', d, 'first-negative-line', neg, 'paren', p)
if bad_strings:
    print('BROKEN STRING LITERALS (a real newline where \\n belongs):')
    for n, t in bad_strings[:10]:
        print('  line %u: %s' % (n, t))
if '--require' in sys.argv:
    want = sys.argv[sys.argv.index('--require') + 1:]
    for t in want:
        print(('HIT ' if t in s else 'MISS'), t)
    missing = [t for t in want if t not in s]
    if missing:
        print('MISSING ANCHORS:', ', '.join(missing))

fail = bool(bad_strings) or d != 0 or p != 0 or ('--require' in sys.argv and
       any(t not in s for t in sys.argv[sys.argv.index('--require') + 1:]))
sys.exit(1 if fail else 0)
