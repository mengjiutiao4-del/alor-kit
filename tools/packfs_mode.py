import struct, sys, stat

SUPER = struct.Struct('<8sIIQQQQQQ')
DNODE = struct.Struct('<IIIIQQIIIIQQ')
P = sys.argv[1]
data_needle = sys.argv[2].encode() if len(sys.argv) > 2 else b'launchd'
fix = '--fix' in sys.argv

with open(P, 'r+b') as f:
    head = f.read(4096)
    magic, block, count, nodes_off, strings_off, strings_len, data_off, image_size, mtime = SUPER.unpack(head[:SUPER.size])
    assert magic == b'PACKFS1\0', magic
    print('nodes', count, 'strings_off', strings_off, 'data_off', data_off, 'image', image_size)
    f.seek(nodes_off)
    raw = bytearray(f.read(count * DNODE.size))
    f.seek(strings_off)
    strings = f.read(strings_len)
    names = {}
    for i in range(count):
        fields = DNODE.unpack_from(raw, i * DNODE.size)
        mode, uid, gid, nlink, size, data, parent, name_off, name_len, rsv, nmtime, flags = fields
        name = strings[name_off:name_off + name_len]
        names[i] = (name, parent)
    for i, (name, parent) in names.items():
        if data_needle in name:
            full = []
            j = i
            while j is not None:
                n, p = names[j]
                full.append(n.decode('latin1'))
                j = p if p != j else None
            fields = list(DNODE.unpack_from(raw, i * DNODE.size))
            print(i, 'mode=%o' % fields[0], 'uid', fields[1], 'size', fields[4], '/'.join(reversed(full)))
            if fix:
                fields[0] = 0o100755
                DNODE.pack_into(raw, i * DNODE.size, *fields)
    if fix:
        f.seek(nodes_off)
        f.write(raw)
        print('written')
