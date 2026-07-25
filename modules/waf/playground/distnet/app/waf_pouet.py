# Module exported and used for configuring the package pouet
#
# To try the changes, you may have to remove the cache and the existing package, for example:
# $ rm -rf /home/user/waf/playground/distnet/packages/app/1.0.0 /tmp/distnetcache/app/1.0.0
# $ waf configure_all build_all package publish

import os

def options(opt):
	# project-specific options go here
	pass

def configure(conf):
	pass
	# one possibility is to specify the configuration variables explicitly:
	# conf.env.append_value('DEFINES_pouet', 'pouet=1')
	# conf.env.append_value('INCLUDES_pouet', os.path.dirname(os.path.abspath(__file__)))
	# conf.env.append_value('LIB_pouet', ['prepouet', 'pouet'])

	if conf.variant == 'linux_64_release':
		# the other project will get -lm in the variant
		conf.env.LIB_m = ['m']
	conf.env.LIB_prepouet = 'prepouet'

def build(bld):
	# another possibility is to create a fake library
	noarch = os.path.dirname(os.path.abspath(__file__))
	base = os.path.dirname(noarch)
	p = os.path.join(base, bld.variant)
	tg = bld.read_shlib(name='pouet', paths=[p])
	tg.export_defines = 'pouet=1'
	tg.export_includes = noarch
	tg.use = 'prepouet m'.split()

	# and again, you have the choice of making fake libraries, or to use variables
	#tg2 = bld.read_shlib(name='prepouet', paths=[p])
	#tg2.use = 'm'

