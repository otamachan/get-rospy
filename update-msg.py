import glob
import os
import sys

MESSAGES = [
    # List of (package, [depend_package1, depend_pacakage2, ...]),
    ('std_msgs', []),
    ('roscpp', []),
    ('rosgraph_msgs', ['std_msgs']),
]

if os.path.basename(sys.executable) == 'Pythonista':
    DEST_DIR = os.path.join(os.path.expanduser('~'), 'Documents', 'site-packages')
else:
    DEST_DIR = os.path.join(os.getcwd(), 'site-packages')


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

if not DEST_DIR in sys.path:
    sys.path.append(DEST_DIR)
for package, includes in MESSAGES:
    generate(package, includes)

print "done"
