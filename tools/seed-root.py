#!/usr/bin/env python3
"""Seed root.packfs with the data-volume directories iOS expects but never creates here.

Measured against work/images/root.packfs: var/mobile/Library/Preferences already carries the
upstream tweaks (BKNoWatchdogs, SetupDone, LastSystemVersion), but var/containers holds only
Bundle/Data/Shared -- no Data/System, no Shared/SystemGroup, no Temp -- and no mobileactivationd
container. backboardd then aborts on
containerURLForSecurityApplicationGroupIdentifier(@"com.apple.security.system-groups") returning
nil ([NSURL initFileURLWithPath:] with a nil path), and launchd's _PanicOnCrash turns that daemon
crash into a kernel panic, which is the 900E dump state we keep landing in.

Rewrites the image in place with the node/string/data mechanics of stage-macan-adsp.py, so no
multi-GB rootfs scan is needed. New files are stored raw (flags 0), the way /fw already is.
Existing files are decoded through the chunk table, merged, and re-stored raw.
The original image is kept beside it as <name>.stock.
"""

import os
from pathlib import Path
import plistlib
import shutil
import stat
import struct
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mkpackfs import SUPER, DNODE, MAGIC, SUPER_SIZE, align

ROOT = Path(__file__).resolve().parents[1]
BUILD = '20H392'
VERSION = '16.7.16'
MOBILE = 501
MCMD_UUID = '4C3A9E57-2A4B-4E0F-9C71-0D6B5A8E1F22'

DIRS = [
    ('var/containers/Bundle', 33, 33, 0o755),
    ('var/containers/Data/System', 0, 0, 0o755),
    ('var/containers/Data/System/' + MCMD_UUID, MOBILE, MOBILE, 0o755),
    ('var/containers/Shared/SystemGroup', 0, 0, 0o755),
    ('var/containers/Temp', 0, 0, 0o1777),
    ('var/keybags', 0, 0, 0o700),
]

PREFS = {
    # The shipped system is 16.7.16/20H392 while the root's pref still says 20F66; that
    # mismatch is what makes launchd run DataMigration for an hour on every boot, so the
    # pref is pinned to the build we actually ship (BUILD is re-read from system.packfs).
    'com.apple.migration': {
        'LastSystemVersion': BUILD,
        'DMUserDataDisposition': {'SyntheticDidUpgrade': False, 'buildVersion': BUILD},
    },
    'com.apple.springboard': {
        'BKNoWatchdogs': True,
        'FBLaunchWatchdogExceptions': {'com.apple.purplebuddy': 100.0},
        'MCForceShowCloudConfigurationUI': False,
    },
    'com.apple.purplebuddy': {
        'SetupDone': True, 'SetupFinishedAllSteps': True, 'SetupVersion': 11,
        'daemonizeNeedsToRun': False,
    },
    'com.apple.purplebuddy.notbackedup': {
        key: False for key in ('ShouldRunDiagnosticsMiniBuddy', 'ShouldRunHSA2MiniBuddy',
                               'ShouldRunKeychainSyncMiniBuddy', 'ShouldRunMesaMiniBuddy',
                               'ShouldRunPaymentMiniBuddy', 'ShouldRunStandardMiniBuddy',
                               'SilentICDPUpgrade')
    },
    'com.apple.managedconfiguration.notbackedup': {
        'LockdownActivationIndicatesCloudConfigurationAvailable': False,
    },
}

EXTRA_FILES = {
    'var/containers/Data/System/%s/.com.apple.mobile_container_manager.metadata.plist' % MCMD_UUID:
        {'MCMMetadataIdentifier': 'com.apple.mobileactivationd', 'MCMMetadataOwner': MOBILE},
    'var/containers/Data/System/%s/Library/internal/data_ark.plist' % MCMD_UUID: {
        '-ActivationState': 'Activated', '-BrickState': False,
        '-BuildVersion': BUILD, '-ProductVersion': VERSION,
        '-DataMigrationCompleted': True, '-PreSydroDataMigrationCompleted': True,
    },
}


# Five jobs in this root carry launchd's private _PanicOnCrash blob; containermanagerd and
# mobile.usermanagerd panic the whole kernel after two crashes inside 60 s. containermanagerd is
# the daemon that owns the containers above, so one crash loop there is our recurring 900E dump.
# Upstream's lineage disables the blob by renaming the key, which leaves everything else intact.
RENAME_JOBS = ['com.apple.containermanagerd.plist', 'com.apple.mobile.usermanagerd.plist',
               'com.apple.trustd.plist', 'com.apple.remoted.plist',
               'com.apple.dt.remotepairingdeviced.plist']


def load(image):
    with image.open('rb') as f:
        super_raw = bytearray(f.read(SUPER_SIZE))
        (magic, block, count, nodes_off, strings_off, strings_size, data_off, image_size,
         created) = SUPER.unpack_from(super_raw)
        assert magic == MAGIC, magic
        assert image_size == image.stat().st_size, (image_size, image.stat().st_size)
        f.seek(strings_off)
        strings = f.read(strings_size)
        f.seek(nodes_off)
        nodes = []
        for _ in range(count):
            fields = list(DNODE.unpack(f.read(DNODE.size)))
            name = strings[fields[7]:fields[7] + fields[8]]
            target = strings[fields[5]:fields[5] + fields[4]] if stat.S_ISLNK(fields[0]) else None
            nodes.append(dict(fields=fields, name=name, target=target, children={}))
    for i, node in enumerate(nodes[1:], 1):
        parent = node['fields'][6]
        assert 0 <= parent < count and stat.S_ISDIR(nodes[parent]['fields'][0])
        nodes[parent]['children'][node['name']] = node
    nodes[0]['parent'] = nodes[0]
    return super_raw, block, data_off, nodes


def node_new(name, mode, uid, gid, now):
    return dict(fields=[mode, uid, gid, 1, 0, 0, 0, 0, len(name), 0, now, 0],
                name=name, target=None, children={}, parent=None)


def ensure_dir(nodes, path, uid, gid, mode, now, force=True):
    node = nodes[0]
    made = []
    for part in path.split('/'):
        child = node['children'].get(part.encode())
        if child is None:
            child = node_new(part.encode(), stat.S_IFDIR | 0o755, 0, 0, now)
            child['parent'] = node
            node['children'][child['name']] = child
            made.append(part)
        elif not stat.S_ISDIR(child['fields'][0]):
            raise SystemExit('seed: %s/%s is not a directory in the image' % (path, part))
        node = child
    if force:
        node['fields'][0] = stat.S_IFDIR | mode
        node['fields'][1], node['fields'][2] = uid, gid
    return node, made


def _unzstd(chunk):
    global _zstd_dc
    if '_zstd_dc' not in globals():
        try:
            import zstandard
            _zstd_dc = zstandard.ZstdDecompressor()
        except ImportError:
            _zstd_dc = None
    if _zstd_dc is None:
        return chunk
    import io
    try:
        return _zstd_dc.stream_reader(io.BytesIO(chunk)).read()
    except Exception:
        return chunk


def read_regular(image, node):
    """Decode a file node, following the chunk table when the packfs stored it compressed."""
    size, data, flags = node['fields'][4], node['fields'][5], node['fields'][11]
    with image.open('rb') as f:
        if not (flags & 0x1):
            f.seek(data)
            return f.read(size)
        f.seek(data)
        head = struct.unpack('<I', f.read(4))[0]
        count = head // 4
        f.seek(data)
        table = struct.unpack('<%dI' % count, f.read(4 * count))
        body = f.read(table[-1] - head)
    out = b''
    for i in range(count - 1):
        chunk = body[table[i] - head:table[i + 1] - head]
        out += _unzstd(chunk) if len(chunk) < 65536 else chunk
    return out[:size]


def system_build(system):
    """Read ProductVersion/BuildVersion out of system.packfs so the seeded state matches
    the root we actually ship, not whatever build the upstream notes were written on."""
    super_raw, block, data_off, nodes = load(system)
    node = nodes[0]
    for part in 'System/Library/CoreServices/SystemVersion.plist'.split('/'):
        child = node['children'].get(part.encode())
        if child is None:
            return None
        node = child
    values = plistlib.loads(read_regular(system, node))
    return values.get('ProductVersion'), values.get('BuildVersion')


def main():
    image = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'work/images/root.packfs'
    assert image.is_file(), image
    global BUILD, VERSION
    system = ROOT / 'work/images/system.packfs'
    if system.is_file():
        found = system_build(system)
        if found and found[0] and found[1]:
            VERSION, BUILD = found
        print('system volume: %s (%s)' % (VERSION, BUILD))
    super_raw, block, data_off, nodes = load(image)
    now = int(time.time())



    for path, uid, gid, mode in DIRS:
        for target in (path, 'private/' + path):
            _, made = ensure_dir(nodes, target, uid, gid, mode, now)
            print('dir  %-58s %s %d:%d' % (target, 'created' if made else 'present', uid, gid))
    stamp = Path(tempfile.mkdtemp(prefix='seed-tweak-', dir=str(image.parent)))
    added = []
    try:
        def put(path, values, uid, gid, mode):
            # /var and /private/var are two separate trees in this image: every plist has to
            # exist in both, or the daemon that asks by the other name sees nothing.
            base = {}
            node = nodes[0]
            for part in path.split('/'):
                node = node['children'].get(part.encode())
                if node is None:
                    break
            if node is not None and stat.S_ISREG(node['fields'][0]):
                base = plistlib.loads(read_regular(image, node))
                verb = 'merge'
            else:
                verb = 'file '
            base.update(values)
            blob = plistlib.dumps(base, fmt=plistlib.FMT_BINARY)
            for target in (path, 'private/' + path):
                parts = target.split('/')
                parent, _ = ensure_dir(nodes, '/'.join(parts[:-1]), MOBILE, MOBILE, 0o755, now,
                                       force=False)
                key = parts[-1].encode()
                source = stamp / ('n%d.bin' % len(added))
                source.write_bytes(blob)
                file_node = node_new(key, stat.S_IFREG | mode, MOBILE, MOBILE, now)
                file_node['parent'] = parent
                file_node['source'] = source
                parent['children'][key] = file_node
                added.append(file_node)
            print('%s %-56s + twin /private/  %d bytes' % (verb, path, len(blob)))

        for name, values in PREFS.items():
            put('var/mobile/Library/Preferences/%s.plist' % name, values, MOBILE, MOBILE, 0o600)
        for path, values in EXTRA_FILES.items():
            put(path, values, MOBILE, MOBILE, 0o600)

        for tree, other in (('var', 'private/var'), ('private/var', 'var')):
            src = nodes[0]
            dst = nodes[0]
            for part in (tree + '/mobile/Library/Preferences').split('/'):
                src = src['children'].get(part.encode()) if src is not None else None
            for part in (other + '/mobile/Library/Preferences').split('/'):
                dst = dst['children'].get(part.encode()) if dst is not None else None
            if src is None or dst is None:
                continue
            for key, node in sorted(src['children'].items()):
                if key in dst['children']:
                    continue
                blob = read_regular(image, node)
                source = stamp / ('c%d.bin' % len(added))
                source.write_bytes(blob)
                copy = node_new(key, stat.S_IFREG | (node['fields'][0] & 0o7777), MOBILE, MOBILE, now)
                copy['parent'] = dst
                copy['source'] = source
                dst['children'][key] = copy
                added.append(copy)
                print('copy %-40s -> %s' % (key.decode('latin1'), other))

        renamed = 0
        jobs_dir = nodes[0]['children'].get(b'LaunchDaemons')
        assert jobs_dir is not None and jobs_dir['children'], 'no LaunchDaemons in the image'
        for name in RENAME_JOBS:
            key = name.encode()
            old = jobs_dir['children'].get(key)
            data = plistlib.loads(read_regular(image, old)) if (
                old is not None and old['fields'][4]) else {}
            if old is None or '_PanicOnCrash' not in data:
                done = '_PanicOnCrasX' in data
                print('job   %-44s %s' % (name, 'already disabled' if done else 'no _PanicOnCrash'))
                if done:
                    renamed += 1
                    continue
                raise SystemExit('seed: %s is not in the image' % name)
            data['_PanicOnCrasX'] = data.pop('_PanicOnCrash')
            source = stamp / ('job%d.bin' % len(added))
            source.write_bytes(plistlib.dumps(data, fmt=plistlib.FMT_BINARY))
            node = node_new(key, stat.S_IFREG | (old['fields'][0] & 0o7777), old['fields'][1],
                            old['fields'][2], now)
            node['parent'] = jobs_dir
            node['source'] = source
            jobs_dir['children'][key] = node
            added.append(node)
            renamed += 1
            print('job   %-44s _PanicOnCrash renamed' % name)
        if renamed != len(RENAME_JOBS):
            raise SystemExit('seed: only %d of %d panic jobs found; the image changed under us'
                             % (renamed, len(RENAME_JOBS)))
        # mobile owns its own library tree, and the activation container is mobile's
        for path, mode in (('var/mobile', 0o751), ('var/mobile/Library', 0o755),
                           ('var/mobile/Library/Preferences', 0o755),
                           ('var/containers/Data/System/%s/Library' % MCMD_UUID, 0o755),
                           ('var/containers/Data/System/%s/Library/internal' % MCMD_UUID, 0o755)):
            ensure_dir(nodes, path, MOBILE, MOBILE, mode, now)

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

        stock = image.with_name(image.name + '.stock')
        if not stock.exists():
            shutil.copyfile(image, stock)
            print('backup', stock.name, stock.stat().st_size)

        fd, temp = tempfile.mkstemp(prefix=image.name + '.seed-', dir=str(image.parent))
        with os.fdopen(fd, 'w+b') as out, image.open('rb') as old:
            shutil.copyfileobj(old, out, 8 * 1024 * 1024)
            for node in added:
                out.seek(align(out.tell(), block))
                node['fields'][5] = out.tell()
                node['fields'][4] = node['source'].stat().st_size
                with node['source'].open('rb') as source:
                    shutil.copyfileobj(source, out)
            new_nodes = align(out.tell(), 4096)
            out.seek(new_nodes)
            for node in ordered:
                out.write(DNODE.pack(*node['fields']))
            new_strings = align(out.tell(), 4096)
            out.seek(new_strings)
            out.write(strings)
            end = align(out.tell(), block)
            out.truncate(end)
            SUPER.pack_into(super_raw, 0, MAGIC, block, len(ordered), new_nodes, new_strings,
                            len(strings), data_off, end, now)
            out.seek(0)
            out.write(super_raw)
            out.flush()
            os.fsync(out.fileno())
        os.chmod(temp, image.stat().st_mode & 0o777)

        with open(temp, 'rb') as f:
            for node in added:
                f.seek(node['fields'][5])
                assert f.read(node['fields'][4]) == node['source'].read_bytes(), node['name']
        os.replace(temp, image)
    finally:
        shutil.rmtree(stamp, ignore_errors=True)
    print('seeded %s: %d -> %d nodes, %s bytes' % (image.name, len(nodes), len(ordered),
                                                   image.stat().st_size))


if __name__ == '__main__':
    main()
