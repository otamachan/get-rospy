import glob
import os
import sys
import urllib2
import zipfile
PACKAGES = {
    'catkin': ('ros/catkin', '0.7.1', 'python/catkin', []),
    'catkin_pkg': ('ros-infrastructure/catkin_pkg', '0.2.10', 'src/catkin_pkg', []),
    'genmsg': ('ros/genmsg', '0.5.7', 'src/genmsg', []),
    'genpy': ('ros/genpy', '0.5.8', 'src/genpy', []),
    'rosgraph': ('ros/ros_comm', '1.12.0', 'tools/rosgraph/src/rosgraph', []),
    'roslib': ('ros/ros', '1.13.1', 'core/roslib/src/roslib', []),
    'rospkg': ('ros-infrastructure/rospkg', '1.0.38', 'src/rospkg', []),
    'rospy': ('ros/ros_comm', '1.12.0', 'clients/rospy/src/rospy', []),
    'std_msgs': ('ros/std_msgs', '0.5.10', '', ['msg']),
    'roscpp': ('ros/ros_comm', '1.12.0', 'clients/roscpp', ['msg', 'srv']),
    'rosgraph_msgs': ('ros/ros_comm_msgs', '1.11.2', 'rosgraph_msgs', ['msg']),
}
MESSAGES = [
    ('std_msgs', []),
    ('roscpp', []),
    ('rosgraph_msgs', ['std_msgs']),
]
if os.path.basename(sys.executable) == 'Pythonista':
    DEST_DIR = os.path.join(os.path.expanduser('~'), 'Documents', 'site-packages')
else:
    DEST_DIR = os.path.join(os.getcwd(), 'site-packages')
TMP_DIR = os.environ.get('TMPDIR', os.environ.get('TMP', '/tmp'))


def download_from_github(repo, ver):
    url = 'https://github.com/{0}/archive/{1}.zip'.format(repo, ver)
    fname = '{0}_{1}.zip'.format(repo.split('/')[1], ver)
    zip_file = os.path.join(TMP_DIR, fname)
    if not os.path.exists(zip_file):
        print "Downloading {0} ...".format(fname)
        try:
            u = urllib2.urlopen(url)
            with open(zip_file, 'wb') as outs:
                block_sz = 8192
                while True:
                    buf = u.read(block_sz)
                    if not buf:
                        break
                    outs.write(buf)
        except:
            sys.stderr.write('Download failed!\n')
            sys.exit(1)
    return zip_file


def unzip(zip_file, dest, path, pattern):
    print "Unzipping into {0} ...".format(dest)
    full_dest_dir = os.path.join(DEST_DIR, dest)
    if not os.path.exists(full_dest_dir):
        os.makedirs(full_dest_dir)
    with open(zip_file, 'rb') as ins:
        try:
            zipfp = zipfile.ZipFile(ins)
            for original_name in zipfp.namelist():
                name = original_name.split('/', 1)[1]
                if path:
                    if not name.startswith(path):
                        continue
                    name = name[len(path)+1:]
                if pattern:
                    for p in pattern:
                        if name.startswith(p+'/'):
                            break
                    else:
                        continue
                fname = os.path.join(full_dest_dir, name)
                data = zipfp.read(original_name)
                if fname.endswith('/'):
                    if not os.path.exists(fname):
                        os.makedirs(fname)
                else:
                    fp = open(fname, 'wb')
                    try:
                        fp.write(data)
                    finally:
                        fp.close()
        except:
            sys.stderr.write('The zip file is corrupted.\n')
            sys.exit(1)


def generate(package, includes):
    import genpy.genpy_main
    import genpy.generator
    print "Generating {0} ...".format(package)
    package_path = os.path.join(DEST_DIR, package)
    for gentype in ('msg', 'srv'):
        files = glob.glob(os.path.join(package_path, gentype,
                                       '*.{0}'.format(gentype)))
        if files:
            if gentype == 'msg':
                gen = genpy.generator.MsgGenerator()
            elif gentype == 'srv':
                gen = genpy.generator.SrvGenerator()
            search_path = {}
            for include in set(includes+[package]):
                search_path[include] = [os.path.join(DEST_DIR,
                                                     include, 'msg')]
            ret = gen.generate_messages(package,
                                        files,
                                        os.path.join(package_path, gentype),
                                        search_path)
            if ret:
                sys.stderr.write('Failed to generate python files from msg files.\n')
                sys.exit(1)
            genpy.generate_initpy.write_modules(os.path.join(package_path, gentype))
        genpy.generate_initpy.write_modules(os.path.join(package_path))

for package, package_info in PACKAGES.iteritems():
    repo, ver, path, pattern = package_info
    zip_file = download_from_github(repo, ver)
    unzip(zip_file, package, path, pattern)
if not DEST_DIR in sys.path:
    sys.path.append(DEST_DIR)
for package, includes in MESSAGES:
    generate(package, includes)

print "done"
