import sys
import re

def strip_comments(src: str) -> str:
    out = []
    i = 0
    n = len(src)
    state = 'code'  # code, line_comment, block_comment, string, char
    while i < n:
        c = src[i]
        nc = src[i+1] if i + 1 < n else ''

        if state == 'code':
            if c == '/' and nc == '/':
                state = 'line_comment'
                i += 2
                continue
            elif c == '/' and nc == '*':
                state = 'block_comment'
                i += 2
                continue
            elif c == '"':
                out.append(c)
                state = 'string'
                i += 1
                continue
            elif c == "'":
                out.append(c)
                state = 'char'
                i += 1
                continue
            else:
                out.append(c)
                i += 1
                continue

        elif state == 'line_comment':
            if c == '\n':
                out.append(c)
                state = 'code'
            i += 1
            continue

        elif state == 'block_comment':
            if c == '*' and nc == '/':
                state = 'code'
                i += 2
                continue
            elif c == '\n':
                out.append(c)  # preserve line breaks for line numbering
                i += 1
                continue
            else:
                i += 1
                continue

        elif state == 'string':
            out.append(c)
            if c == '\\' and i + 1 < n:
                out.append(src[i+1])
                i += 2
                continue
            elif c == '"':
                state = 'code'
            i += 1
            continue

        elif state == 'char':
            out.append(c)
            if c == '\\' and i + 1 < n:
                out.append(src[i+1])
                i += 2
                continue
            elif c == "'":
                state = 'code'
            i += 1
            continue

    result = ''.join(out)
    # Collapse lines that are now blank due to removed comments, but keep at most one blank line in a row
    lines = result.split('\n')
    cleaned = []
    prev_blank = False
    for line in lines:
        stripped = line.rstrip()
        is_blank = (stripped.strip() == '')
        if is_blank and prev_blank:
            continue
        cleaned.append(stripped)
        prev_blank = is_blank
    return '\n'.join(cleaned) + '\n'

if __name__ == '__main__':
    infile, outfile = sys.argv[1], sys.argv[2]
    with open(infile, 'r') as f:
        src = f.read()
    result = strip_comments(src)
    with open(outfile, 'w') as f:
        f.write(result)
