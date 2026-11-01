#!/usr/bin/env python3
"""Seed /private/var the way this stack actually seeds it: through /etc/var.skel.

userland/src/init.c mount_writable() does mount("ramfs", "/private/var", 0, NULL) -- a brand new
empty writable filesystem -- and only then replays /etc/var.skel from the image:

    d <mode8> <uid> <gid> <path>              mkdir + chmod + chown
    l <target> <path>                         symlink
    f <mode8> <uid> <gid> <path> <image-path> copy_file(in_image(<image-path>), <path>)

Anything baked into /private/var inside the system volume is therefore invisible: the ramfs
covers it. That is why the containers had to be declared here instead. init.c reads the skeleton
into a 16 KB buffer, so the file has to stay under that, and parents must be listed before
children because mkdir is not recursive and one failure stops the replay.

Writes the new /etc/var.skel into root.packfs with the same tail-append trick used by
seed-group-containers.py (existing file bytes never move).
"""

from pathlib import Path
import os
import stat
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import importlib.util
spec = importlib.util.spec_from_file_location('seedroot', str(HERE / 'seed-root.py'))
seedroot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(seedroot)
leaf = importlib.util.spec_from_file_location('leaves', str(HERE / 'seed-group-containers.py'))
leaves = importlib.util.module_from_spec(leaf)

load = seedroot.load
align = seedroot.align
DNODE = seedroot.DNODE
SUPER = seedroot.SUPER
MAGIC = seedroot.MAGIC
node_new = seedroot.node_new

IMAGE = HERE.parent / 'work/images/root.packfs'
LIMIT = 16384
GROUP_UUID = '9F1D7B46-5C28-4A0E-8B13-72E4D60A3C55'
ARK_UUID = '4C3A9E57-2A4B-4E0F-9C71-0D6B5A8E1F22'
PREFS = ('com.apple.springboard', 'com.apple.migration', 'com.apple.purplebuddy',
         'com.apple.purplebuddy.notbackedup', 'com.apple.managedconfiguration.notbackedup')

# parents first, alphabetical, exactly the order the replay wants
ADDED = []
for path, uid, gid, mode in (
        ('/private/var/containers/Bundle', 33, 33, '755'),
        ('/private/var/containers/Data', 0, 0, '755'),
        ('/private/var/containers/Data/System', 0, 0, '755'),
        ('/private/var/containers/Shared', 0, 0, '755'),
        ('/private/var/containers/Shared/SystemGroup', 0, 0, '755'),
        ('/private/var/containers/Temp', 0, 0, '1777'),
        ('/private/var/keybags', 0, 0, '700'),
        ('/private/var/mobile', 501, 501, '755'),
        ('/private/var/mobile/Library', 501, 501, '755'),
        ('/private/var/mobile/Library/Preferences', 501, 501, '755'),
        ('/private/var/mobile/Library/Logs', 501, 501, '755'),
        ('/private/var/containers/Data/System/' + ARK_UUID, 501, 501, '755'),
        ('/private/var/containers/Data/System/' + ARK_UUID + '/Library', 501, 501, '755'),
        ('/private/var/containers/Data/System/' + ARK_UUID + '/Library/internal', 501, 501, '755'),
        ('/private/var/containers/Shared/SystemGroup/' + GROUP_UUID, 501, 501, '755'),
        ('/private/var/containers/Shared/SystemGroup/' + GROUP_UUID + '/Library', 501, 501, '755')):
    ADDED.append('d\t%s\t%d\t%d\t%s' % (mode, uid, gid, path))

# the tuned preferences and the activation container come straight out of this same image,
# where last night's seeding put them under /var/...
for name in PREFS:
    ADDED.append('f\t600\t501\t501\t/private/var/mobile/Library/Preferences/%s.plist'
                 '\t/var/mobile/Library/Preferences/%s.plist' % (name, name))
ADDED.append('f\t600\t501\t501\t/private/var/containers/Data/System/%s/'
             '.com.apple.mobile_container_manager.metadata.plist'
             '\t/var/containers/Data/System/%s/.com.apple.mobile_container_manager.'
             'metadata.plist' % (ARK_UUID, ARK_UUID))
ADDED.append('f\t600\t501\t501\t/private/var/containers/Data/System/%s/Library/internal/'
             'data_ark.plist\t/var/containers/Data/System/%s/Library/internal/'
             'data_ark.plist' % (ARK_UUID, ARK_UUID))
ADDED.append('f\t600\t501\t501\t/private/var/containers/Shared/SystemGroup/%s/'
             '.com.apple.mobile_container_manager.metadata.plist'
             '\t/private/var/containers/Shared/SystemGroup/%s/'
             '.com.apple.mobile_container_manager.metadata.plist' % (GROUP_UUID, GROUP_UUID))


def main():
    dry = '--dry' in sys.argv
    super_raw, block, data_off, nodes = load(IMAGE)
    now = int(time.time())
    etc = nodes[0]['children'].get(b'etc')
    assert etc is not None, 'no etc/ in the image'
    old = etc['children'].get(b'var.skel')
    assert old is not None, 'no etc/var.skel'
    text = seedroot.read_regular(IMAGE, old).decode('utf-8')
    lines = [l for l in text.splitlines() if l.strip()]
    present = {l.split('	')[4] for l in lines if l.startswith('d	')}
    # an 'f' line from an earlier pass may carry the wrong source column (init.c prepends the
    # /redwood mount point itself), so those are always re-emitted and the stale copy dropped
    mine = {l.split('	')[4] for l in ADDED if l.startswith('f	')}
    lines = [l for l in lines if not (l.startswith('f	') and l.split('	')[4] in mine)]
    new = [l for l in ADDED if l.split('	')[4] not in present]
    print('var.skel now: %d lines %d bytes; %d to add' % (len(lines), len(text), len(new)))
    for l in new:
        print('  +', l.replace('\t', ' | '))
    blob = ('\n'.join(lines + new) + '\n').encode('utf-8')
    if len(blob) >= LIMIT:
        raise SystemExit('seed: /etc/var.skel would be %d bytes and init.c reads only %d'
                         % (len(blob), LIMIT))
    # every 'f' source has to exist inside this image, or copy_file just fails at boot
    missing = []
    for l in new:
        f = l.split('\t')
        if f[0] != 'f':
            continue
        node = nodes[0]
        for part in f[5].lstrip('/').split('/'):
            node = node['children'].get(part.encode())
            if node is None:
                missing.append(f[5])
                break
    if missing:
        raise SystemExit('seed: these sources are not in the image: %s' % missing)
    print('all %d file sources present' % sum(1 for l in new if l.startswith('f')))
    if dry:
        print('dry run: nothing written')
        return

    node = node_new(b'var.skel', stat.S_IFREG | 0o644, 0, 0, now)
    node['blob'] = blob
    node['parent'] = etc
    etc['children'][b'var.skel'] = node
    ordered = [nodes[0]]
    nodes[0]['index'] = 0
    for n in ordered:
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
        if n.get('target') is not None:
            fields[5] = len(strings)
            fields[4] = len(n['target'])
            strings += n['target'] + b'\0'
        if stat.S_ISDIR(fields[0]):
            fields[4] = len(n['children'])
            fields[5] = min((c['index'] for c in n['children'].values()), default=0)
            fields[3] = 2 + sum(stat.S_ISDIR(c['fields'][0]) for c in n['children'].values())
    with IMAGE.open('r+b') as f:
        f.seek(0, os.SEEK_END)
        pos = align(f.tell(), block)
        f.truncate(pos)
        f.seek(pos)
        node['fields'][5] = pos
        node['fields'][4] = len(blob)
        f.write(blob)
        new_nodes = align(f.tell(), 4096)
        f.truncate(new_nodes)
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
    print('wrote /etc/var.skel: %d bytes; %s now %d nodes %d bytes'
          % (len(blob), IMAGE.name, len(ordered), IMAGE.stat().st_size))


if __name__ == '__main__':
    main()
