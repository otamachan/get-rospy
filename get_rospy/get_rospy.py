# -*- codeing: utf-8 -*-
from __future__ import print_function
import glob
import os
import sys
try:
    from urllib2 import urlopen
except:
    from urllib.request import urlopen
import zipfile
_INSTALL_PACKAGES = [
    ('otamachan/get-rospy', 'master', 'get_rospy'),
    ('ros/catkin', '0.7.6', 'python/catkin'),
    ('ros-infrastructure/catkin_pkg', '0.3.9', 'src/catkin_pkg'),
    ('ros/genmsg', '0.5.8', 'src/genmsg'),
    ('ros/genpy', '0.6.5', 'src/genpy'),
    ('ros/ros_comm', '1.12.7', 'tools/rosgraph/src/rosgraph'),
    ('ros/ros', '1.13.5', 'core/roslib/src/roslib'),
    ('ros-infrastructure/rospkg', '1.1.4', 'src/rospkg'),
    ('ros/ros_comm', '1.12.7', 'clients/rospy/src/rospy'),
]
_INSTALL_MESSAGES = [
    ('ros/std_msgs', '0.5.11', '', []),
    ('ros/ros_comm', '1.12.7', 'clients/roscpp', []),
    ('ros/ros_comm_msgs', '1.11.2', 'rosgraph_msgs', ['std_msgs']),
]

if os.path.basename(sys.executable) == 'Pythonista':
    DEST_DIR = os.path.join(os.path.expanduser('~'), 'Documents', 'site-packages')
else:
    DEST_DIR = os.path.join(os.getcwd(), 'site-packages')
TMP_DIR = os.environ.get('TMPDIR', os.environ.get('TMP', '/tmp'))


def get_package(repo, path):
    if path:
        return path.rsplit('/', 1)[-1]
    else:
        return repo.split('/')[1]


def download_from_github(repo, ver):
    url = 'https://github.com/{0}/archive/{1}.zip'.format(repo, ver)
    fname = '{0}_{1}.zip'.format(repo.split('/')[1], ver)
    zip_file = os.path.join(TMP_DIR, fname)
    if not os.path.exists(zip_file):
        print("Downloading {0} ...".format(fname))
        try:
            u = urlopen(url)
            with open(zip_file, 'wb') as outs:
                block_sz = 8192
                while True:
                    buf = u.read(block_sz)
                    if not buf:
                        break
                    outs.write(buf)
        except:
            raise RuntimeError('Download failed!')
    return zip_file


def unzip(zip_file, package, path, is_msg):
    print("Unzipping into {0} ...".format(package))
    package_path = os.path.join(DEST_DIR, package)
    if not os.path.exists(package_path):
        os.makedirs(package_path)
    with open(zip_file, 'rb') as ins:
        try:
            zipfp = zipfile.ZipFile(ins)
            for original_name in zipfp.namelist():
                name = original_name.split('/', 1)[1]
                if path:
                    if not name.startswith(path):
                        continue
                    name = name[len(path)+1:]
                if is_msg:
                    if not (name.startswith('msg/') or name.startswith('srv/')):
                        continue
                fname = os.path.join(package_path, name)
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
            raise RuntimeError('The zip file is corrupted.')


def generate(package, includes):
    import genpy.genpy_main
    import genpy.generator
    print("Generating {0} ...".format(package))
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
                raise RuntimeError('Failed to generate python files from msg files.')
            genpy.generate_initpy.write_modules(os.path.join(package_path, gentype))
        genpy.generate_initpy.write_modules(os.path.join(package_path))


def install_messages(messages):
    for repo, ver, path, includes in messages:
        if repo:
            zip_file = download_from_github(repo, ver)
            unzip(zip_file, get_package(repo, path), path, True)
        generate(get_package(repo, path), includes)


def install():
    for repo, ver, path in _INSTALL_PACKAGES:
        zip_file = download_from_github(repo, ver)
        unzip(zip_file, get_package(repo, path), path, False)
    if not DEST_DIR in sys.path:
        sys.path.append(DEST_DIR)
    install_messages(_INSTALL_MESSAGES)
    print("done")


if __name__ == '__main__':
    install()
