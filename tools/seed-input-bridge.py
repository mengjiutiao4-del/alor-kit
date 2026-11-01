#!/usr/bin/env python3
"""Install the touch -> HID bridge into the root volume (which mounts at /redwood).

The kernel only queues FT3683 contacts on /dev/redwood-touch; without a userspace reader the
event server never sees a digitizer.  CI now builds two assets for that reader:
libredwood.dylib (the forwarding thread, started from its library constructor) and
redwood-inputd (a daemon that dlopens it and stays alive).

Tail-append writer shared with the other seeders: existing bytes never move, and the rollback
image must already exist before anything is written.
"""

import hashlib
import os
from pathlib import Path
import plistlib
import stat
import sys
import time
import importlib.util

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
spec = importlib.util.spec_from_file_location('seedroot', str(HERE / 'seed-root.py'))
seedroot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(seedroot)

load = seedroot.load
align = seedroot.align
DNODE = seedroot.DNODE
SUPER = seedroot.SUPER
MAGIC = seedroot.MAGIC
node_new = seedroot.node_new

IMAGE = HERE.parent / 'work/images/root.packfs'
PLIST = {'Label': 'com.redwood.input',
         'Program': '/redwood/bin/redwood-inputd',
         'RunAtLoad': True,
         'KeepAlive': True,
         'StandardOutPath': '/dev/console',
         'StandardErrorPath': '/dev/console'}


def newest_bridge():
    kits = sorted((p for p in HERE.parent.parent.glob('kit[0-9]*')
                   if p.is_dir() and p.name[3:].isdigit()), key=lambda p: int(p.name[3:]))
    for kit in reversed(kits):
        if (kit / 'libredwood.dylib').is_file() and (kit / 'redwood-inputd').is_file():
            return kit
    raise SystemExit('no kit holds both bridge assets yet; run grab_release.sh first')


def walk(root, path):
    node = root
    for part in path.split('/'):
        node = node['children'].get(part.encode())
        if node is None:
            return None
    return node


def main():
    kit = Path(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith('--') else newest_bridge()
    dry = '--dry' in sys.argv
    want = [('bin/redwood-inputd', (kit / 'redwood-inputd').read_bytes(), 0o755),
            ('lib/libredwood.dylib', (kit / 'libredwood.dylib').read_bytes(), 0o644),
            ('LaunchDaemons/com.redwood.input.plist',
             plistlib.dumps(dict(PLIST), fmt=plistlib.FMT_BINARY), 0o644)]
    for name, blob, _ in want:
        head = blob[:4]
        if name.endswith('inputd') or name.endswith('.dylib'):
            magic = int.from_bytes(head[:4], 'little') & 0xfffffffe
            if magic not in (0xfeedfacf, 0xfeedface):
                raise SystemExit('%s is not a mach-o (head %s)' % (name, head.hex()))
        print('want %-40s %8d bytes' % (name, len(blob)))

    assert IMAGE.is_file(), IMAGE
    stock = IMAGE.with_suffix(IMAGE.suffix + '.stock')
    if not stock.is_file():
        raise SystemExit('refusing to rewrite %s: no rollback copy (%s) exists' % (IMAGE.name, stock))
    if dry:
        print('dry run: nothing written to %s' % IMAGE.name)
        return

    super_raw, block, data_off, nodes = load(IMAGE)
    # Measured 2026-11-01: this root already ships /redwood/lib/libredwood.dylib and /etc/inject.conf
    # names it for backboardd, so the touch -> HID path is already live.  Installing the standalone
    # daemon on top would register a second digitizer service for the same events.
    conf = walk(nodes[0], 'etc/inject.conf')
    if conf is not None and b'libredwood.dylib' in seedroot.read_regular(IMAGE, conf):
        print('nothing to do: /etc/inject.conf already injects libredwood.dylib into the GUI servers')
        return
    now = int(time.time())
    fresh = []
    for name, blob, mode in want:
        if walk(nodes[0], name) is not None:
            print('  present %s' % name)
            continue
        parent = nodes[0]
        for part in name.split('/')[:-1]:
            child = parent['children'].get(part.encode())
            if child is None:
                child = node_new(part.encode(), stat.S_IFDIR | 0o755, 0, 0, now)
                child['children'] = {}
                parent['children'][part.encode()] = child
                print('  dir     %s' % part)
            parent = child
        node = node_new(name.split('/')[-1].encode(), stat.S_IFREG | mode, 0, 0, now)
        node['blob'] = blob
        node['parent'] = parent
        parent['children'][node['name']] = node
        fresh.append(node)
        print('  add     %s' % name)
    if not fresh:
        print('bridge already installed')
        return

    ordered = [nodes[0]]
    nodes[0]['index'] = 0
    for n in ordered:
        assert stat.S_ISDIR(n['fields'][0]), 'indexing reached a file'
        for name in sorted(n['children']):
            child = n['children'][name]
            child['parent'] = n
            child['index'] = len(ordered)
            ordered.append(child)
    strings = bytearray(b'\0')
    for n in ordered:
        fields = n['fields']
        fields[6] = n['parent']['index']
        fields[7] = len(strings)
        strings += n['name'] + b'\0'
        if stat.S_ISLNK(fields[0]):
            fields[5] = len(strings)
            strings += n['target'] + b'\0'
        if stat.S_ISDIR(fields[0]):
            fields[4] = len(n['children'])
            fields[5] = min((c['index'] for c in n['children'].values()), default=0)
            fields[3] = 2 + sum(stat.S_ISDIR(c['fields'][0]) for c in n['children'].values())
    for n in ordered:
        if 'blob' in n:
            n['fields'][5] = align(n['fields'][5], block)

    with IMAGE.open('r+b') as f:
        f.seek(0, os.SEEK_END)
        pos = align(f.tell(), block)
        f.truncate(pos)
        for n in ordered:
            if 'blob' not in n:
                continue
            f.seek(pos)
            n['fields'][5] = pos
            n['fields'][4] = len(n['blob'])
            f.write(n['blob'])
            pos = align(f.tell(), block)
            f.truncate(pos)
        new_nodes = pos
        f.seek(new_nodes)
        for n in ordered:
            f.write(DNODE.pack(*n['fields']))
        new_strings = align(f.tell(), 4096)
        f.seek(new_strings)
        f.write(bytes(strings))
        end = align(f.tell(), block)
        f.truncate(end)
        SUPER.pack_into(super_raw, 0, MAGIC, block, len(ordered), new_nodes, new_strings,
                        len(strings), data_off, end, now)
        f.seek(0)
        f.write(bytes(super_raw))
        f.flush()
        os.fsync(f.fileno())

    check, _, _, reread = load(IMAGE)
    bad = []
    for name, blob, mode in want:
        node = walk(reread[0], name)
        if node is None:
            bad.append(name + ': missing')
            continue
        got = seedroot.read_regular(IMAGE, node)
        if got != blob:
            bad.append('%s: sha %s != %s' % (name, hashlib.sha256(got).hexdigest()[:12],
                                             hashlib.sha256(blob).hexdigest()[:12]))
    if bad:
        raise SystemExit('verify failed: %s (restore with the .stock copy)' % bad)
    print('installed %d files; %s is %d nodes %d bytes; every blob read back identical'
          % (len(fresh), IMAGE.name, len(ordered), IMAGE.stat().st_size))


if __name__ == '__main__':
    main()
