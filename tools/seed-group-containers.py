#!/usr/bin/env python3
"""Create the leaf group containers containermanagerd cannot mkdir on a read-only volume.

Last night's seeding made /private/var/containers/Shared/SystemGroup, but that is only the
parent: on a real handset containermanagerd creates one child per group and writes
.com.apple.mobile_container_manager.metadata.plist inside it. Our root and system volumes are
packfs images mounted read-only, so it can never create that child and
containerURLForSecurityApplicationGroupIdentifier(@"com.apple.security.system-groups") still
returns nil -- which is the crash that has been killing backboardd.

This writes the children ahead of time, under both naming conventions iOS is ever seen to use
(a UUID directory holding a metadata plist that names the group, and a directory named after the
group itself), so the lookup succeeds whichever one Foundation asks for.

It appends instead of rewriting: existing file bytes keep their offsets, only the node/string
tables move to the end of the image and the 4 KB superblock is repacked in place. That is the
difference between touching 3 MB and copying 2.4 GB on a disk that is already saturated.
"""

from pathlib import Path
import importlib.util
import os
import plistlib
import stat
import sys
import time

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
SUPER_SIZE = seedroot.SUPER_SIZE
MOBILE = 501

GROUP = 'com.apple.security.system-groups'
GROUP_UUID = '9F1D7B46-5C28-4A0E-8B13-72E4D60A3C55'
CLASS12_UUID = '2C6E48A1-07D3-4F5B-9A16-3E8B0C5D7144'

# (dir path relative to the image root, metadata identifier, owner uid)
LEAVES = [
    ('private/var/containers/Shared/SystemGroup/' + GROUP_UUID, GROUP, MOBILE),
    ('private/var/containers/Shared/SystemGroup/' + GROUP, GROUP, MOBILE),
    ('private/var/containers/Shared/SystemGroup/' + CLASS12_UUID, 'com.apple.springboard', MOBILE),
]
# containermanagerd hands back <container>/Library, so it has to exist too
SUBDIRS = ['Library', 'Library/Caches', 'Documents', 'tmp']


def main():
    image = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE.parent / 'work/images/system.packfs'
    assert image.is_file(), image
    dry = '--dry' in sys.argv
    super_raw, block, data_off, nodes = load(image)
    now = int(time.time())
    print('%s: %d nodes, %d bytes, block %d' % (image.name, len(nodes), image.stat().st_size, block))

    created = []

    def ensure_dir(parts, uid, gid, mode):
        node = nodes[0]
        for part in parts:
            child = node['children'].get(part.encode())
            if child is None:
                child = {'fields': [stat.S_IFDIR | 0o755, 0, 0, 1, 0, 0, 0, 0, len(part.encode()),
                                    0, now, 0], 'name': part.encode(), 'target': None,
                         'children': {}}
                node['children'][child['name']] = child
                created.append(child)
            elif not stat.S_ISDIR(child['fields'][0]):
                raise SystemExit('%s exists and is not a directory' % part)
            node = child
        node['fields'][0] = stat.S_IFDIR | mode
        node['fields'][1], node['fields'][2] = uid, gid
        return node

    def add_file(parent, name, blob, uid, gid, mode):
        key = name.encode()
        old = parent['children'].get(key)
        if old is not None and stat.S_ISREG(old['fields'][0]):
            return None
        node = {'fields': [stat.S_IFREG | mode, uid, gid, 1, len(blob), 0, 0, 0, len(key), 0,
                           now, 0], 'name': key, 'target': None, 'children': {}, 'blob': blob}
        parent['children'][key] = node
        created.append(node)
        return node

    for path, identifier, owner in LEAVES:
        parts = path.split('/')
        container = ensure_dir(parts, owner, owner, 0o755)
        meta = plistlib.dumps({'MCMMetadataIdentifier': identifier, 'MCMMetadataOwner': owner,
                               'MCMMetadataPermissions': 0o755}, fmt=plistlib.FMT_BINARY)
        node = add_file(container, '.com.apple.mobile_container_manager.metadata.plist', meta,
                        owner, owner, 0o600)
        print('container %-70s id %s %s' % ('/'.join(parts[-2:]), identifier,
                                            'exists' if node is None else 'created'))
        for sub in SUBDIRS:
            mode = 0o1777 if sub == 'tmp' else 0o755
            ensure_dir(parts + sub.split('/'), owner, owner, mode)

    if not created:
        print('nothing to do: every leaf already exists')
        return
    print('nodes to add: %d' % len(created))
    if dry:
        print('dry run: nothing written')
        return

    ordered = [nodes[0]]
    nodes[0]['index'] = 0
    for node in ordered:
        for name in sorted(node['children']):
            child = node['children'][name]
            child['parent'] = node
            child['index'] = len(ordered)
            ordered.append(child)
    strings = bytearray(b'\0')
    for node in ordered:
        fields = node['fields']
        fields[6] = node['parent']['index']
        fields[7] = len(strings)
        strings += node['name'] + b'\0'
        if node.get('target') is not None:
            fields[5] = len(strings)
            fields[4] = len(node['target'])
            strings += node['target'] + b'\0'
        if stat.S_ISDIR(fields[0]):
            fields[4] = len(node['children'])
            fields[5] = min((c['index'] for c in node['children'].values()), default=0)
            fields[3] = 2 + sum(stat.S_ISDIR(c['fields'][0]) for c in node['children'].values())

    with image.open('r+b') as f:
        f.seek(0, os.SEEK_END)
        for node in created:
            if 'blob' not in node:
                continue
            pos = align(f.tell(), block)
            f.truncate(pos)
            f.seek(pos)
            node['fields'][5] = pos
            f.write(node['blob'])
        new_nodes = align(f.tell(), 4096)
        f.truncate(new_nodes)
        f.seek(new_nodes)
        for node in ordered:
            f.write(DNODE.pack(*node['fields']))
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
    print('seeded %s: %d nodes, %d bytes' % (image.name, len(ordered), image.stat().st_size))


if __name__ == '__main__':
    main()
