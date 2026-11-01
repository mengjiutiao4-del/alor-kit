#!/usr/bin/env python3
"""Put the data-volume state where iOS actually looks: /private/var on the SYSTEM volume.

Measured tonight: system.packfs is the root filesystem (it owns /System, /Library, and the
"var -> private/var" symlink) while root.packfs is mounted at /redwood -- /System/Library/
LaunchDaemons really is a symlink to /redwood/LaunchDaemons, which is why job plists belong in
the root image. But /private/var in the system volume has no children at all, so
containerURLForSecurityApplicationGroupIdentifier() cannot find Shared/SystemGroup no matter how
many containers we seed under /redwood/var. This script copies the seeded tree from root.packfs
into system.packfs at private/var/, which is the path Foundation asks for.

Rewrites system.packfs in place (2.4 GB), atomically, and keeps a .stock copy of the original.
"""

from pathlib import Path
import importlib.util
import os
import plistlib
import shutil
import stat
import subprocess
import sys
import tempfile
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
spec = importlib.util.spec_from_file_location('seedroot', str(HERE / 'seed-root.py'))
seedroot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(seedroot)

ROOT_IMAGE = HERE.parent / 'work/images/root.packfs'
SYSTEM_IMAGE = HERE.parent / 'work/images/system.packfs'
load = seedroot.load
node_new = seedroot.node_new
read_regular = seedroot.read_regular

# the container skeleton, upstream's list, rooted at private/var
DIRS = ['private/var/containers', 'private/var/containers/Bundle', 'private/var/containers/Data',
        'private/var/containers/Data/System', 'private/var/containers/Shared',
        'private/var/containers/Shared/SystemGroup', 'private/var/containers/Temp',
        'private/var/containers/Bundle/App', 'private/var/keybags', 'private/var/mobile',
        'private/var/mobile/Library', 'private/var/mobile/Library/Preferences',
        'private/var/mobile/Library/Logs', 'private/var/db', 'private/var/db/timezone',
        'private/var/demotion', 'private/var/Keychains', 'private/var/wireless',
        'private/var/wireless/Library', 'private/var/wireless/Library/Preferences',
        'private/var/logs', 'private/var/preferences', 'private/var/MobileDevice',
        'private/var/MobileDevice/ProvisioningProfiles']
PERMS = {'private/var/containers/Bundle': (33, 33, 0o755),
         'private/var/containers/Temp': (0, 0, 0o1777),
         'private/var/keybags': (0, 0, 0o700)}


def main():
    dry = '--dry' in sys.argv
    super_raw, block, data_off, nodes = load(SYSTEM_IMAGE)
    now = int(time.time())
    print('system image: %d nodes, %d bytes' % (len(nodes), SYSTEM_IMAGE.stat().st_size))

    root_spec, b2, d2, root_nodes = load(ROOT_IMAGE)

    def at(tree_nodes, path):
        n = tree_nodes[0]
        for part in path.split('/'):
            n = n['children'].get(part.encode())
            if n is None:
                return None
        return n

    added = []
    for path in DIRS:
        uid, gid, mode = PERMS.get(path, (0, 0, 0o755))
        node = nodes[0]
        made = []
        for part in path.split('/'):
            child = node['children'].get(part.encode())
            if child is None:
                child = node_new(part.encode(), stat.S_IFDIR | 0o755, 0, 0, now)
                node['children'][child['name']] = child
                made.append(part)
            elif not stat.S_ISDIR(child['fields'][0]):
                raise SystemExit('%s/%s exists but is not a directory (mode %s)'
                                 % (path, part, oct(child['fields'][0])))
            node = child
        node['fields'][0] = stat.S_IFDIR | mode
        node['fields'][1], node['fields'][2] = uid, gid
        print('dir  %-56s %s %d:%d' % (path, 'created' if made else 'present', uid, gid))

    # preferences and the activation container come straight from the seeded root image
    for src_dir, dst_dir in (('var/mobile/Library/Preferences',
                             'private/var/mobile/Library/Preferences'),):
        snode = at(root_nodes, src_dir)
        dnode = at(nodes, dst_dir)
        assert snode is not None and dnode is not None, (src_dir, dst_dir)
        for key, sfile in sorted(snode['children'].items()):
            if key in dnode['children']:
                continue
            blob = read_regular(ROOT_IMAGE, sfile)
            plistlib.loads(blob)
            fnode = node_new(key, stat.S_IFREG | (sfile['fields'][0] & 0o7777),
                             sfile['fields'][1], sfile['fields'][2], now)
            fnode['blob'] = blob
            dnode['children'][key] = fnode
            added.append(fnode)
            print('copy %-40s -> /%s' % (key.decode('latin1'), dst_dir))

    uuid_dir = at(nodes, 'private/var/containers/Data/System')
    for key, sfile in sorted(at(root_nodes, 'var/containers/Data/System')['children'].items()):
        if key == b'Data' or not stat.S_ISDIR(sfile['fields'][0]):
            continue
        stack = [(sfile, '', root_nodes)]
        while stack:
            snode, rel, tn = stack.pop()
            for key2, c in sorted(snode['children'].items()):
                if stat.S_ISDIR(c['fields'][0]):
                    stack.append((c, rel + '/' + key2.decode('latin1'), tn))
                else:
                    blob = read_regular(ROOT_IMAGE, c)
                    plistlib.loads(blob)
                    parent = uuid_dir
                    for part in (key.decode('latin1') + rel).split('/'):
                        if not part:
                            continue
                        child = parent['children'].get(part.encode())
                        if child is None:
                            child = node_new(part.encode(), stat.S_IFDIR | 0o755, 501, 501, now)
                            parent['children'][part.encode()] = child
                        parent = child
                    fnode = node_new(key2, stat.S_IFREG | (c['fields'][0] & 0o7777), 501, 501, now)
                    fnode['blob'] = blob
                    parent['children'][key2] = fnode
                    added.append(fnode)
                    print('copy %-40s -> /private/var/containers/Data/System%s/%s'
                          % (key2.decode('latin1'), rel, key.decode('latin1')))

    print('files to add: %d' % len(added))
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
        if node['target'] is not None:
            fields[5] = len(strings)
            strings += node['target'] + b'\0'
        if stat.S_ISDIR(fields[0]):
            fields[4] = len(node['children'])
            fields[5] = min((c['index'] for c in node['children'].values()), default=0)
            fields[3] = 2 + sum(stat.S_ISDIR(c['fields'][0]) for c in node['children'].values())

    stock = SYSTEM_IMAGE.with_name(SYSTEM_IMAGE.name + '.stock')
    if not stock.exists():
        print('copying the untouched system image aside (2.4 GB)...')
        shutil.copyfile(SYSTEM_IMAGE, stock)
        print('backup', stock.name, stock.stat().st_size)

    fd, temp = tempfile.mkstemp(prefix='system.packfs.var-', dir=str(SYSTEM_IMAGE.parent))
    with os.fdopen(fd, 'w+b') as out, SYSTEM_IMAGE.open('rb') as old:
        shutil.copyfileobj(old, out, 8 * 1024 * 1024)
        for node in added:
            out.seek(seedroot.align(out.tell(), block))
            node['fields'][5] = out.tell()
            node['fields'][4] = len(node['blob'])
            out.write(node['blob'])
        new_nodes = seedroot.align(out.tell(), 4096)
        out.seek(new_nodes)
        for node in ordered:
            out.write(seedroot.DNODE.pack(*node['fields']))
        new_strings = seedroot.align(out.tell(), 4096)
        out.seek(new_strings)
        out.write(strings)
        end = seedroot.align(out.tell(), block)
        out.truncate(end)
        seedroot.SUPER.pack_into(super_raw, 0, seedroot.MAGIC, block, len(ordered), new_nodes,
                                 new_strings, len(strings), data_off, end, now)
        out.seek(0)
        out.write(super_raw)
        out.flush()
        os.fsync(out.fileno())
    os.chmod(temp, SYSTEM_IMAGE.stat().st_mode & 0o777)
    with open(temp, 'rb') as f:
        for node in added:
            f.seek(node['fields'][5])
            assert f.read(node['fields'][4]) == node['blob'], node['name']
    os.replace(temp, SYSTEM_IMAGE)
    print('seeded %s: %d nodes, %d bytes' % (SYSTEM_IMAGE.name, len(ordered),
                                             SYSTEM_IMAGE.stat().st_size))


if __name__ == '__main__':
    main()
