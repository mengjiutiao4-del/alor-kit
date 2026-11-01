"""Register the alor touch driver in the kit's input glue (macan_input.c).

A file rather than a workflow heredoc: the inline patches kept turning C
escapes into real newlines.
"""
import sys

path = 'kernel/files/bsd/dev/macan_input.c'
s = open(path, encoding='utf-8', errors='surrogateescape').read()

a = 'extern void macan_adsp_init(void);'
b = a + '\nextern void alor_touch_init(void);'
if b not in s:
    if a not in s:
        sys.exit('ANCHOR MISS: macan_adsp_init declaration')
    s = s.replace(a, b, 1)

a = ' redwood_goodix_init();\n'
b = a + ' alor_touch_init();\n'
if b not in s:
    if a not in s:
        sys.exit('ANCHOR MISS: redwood_goodix_init call')
    s = s.replace(a, b, 1)

open(path, 'w', encoding='utf-8', errors='surrogateescape', newline='').write(s)
print('alor_touch_init registered in macan_input.c')
