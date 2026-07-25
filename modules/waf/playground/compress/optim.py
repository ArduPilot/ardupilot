#! /usr/bin/env python

import os, subprocess, shutil, random, optparse

comp = {
	'bz2': 'cjf',
	'xz' : 'cJf',
	'gz' : 'czf',
}

def read_wafdir():
	try:
		os.listdir('waflib')
	except:
		raise ImportError('please provide a waflib directory in the current folder')

	d = 'waflib'
	lst = [d + os.sep + x for x in os.listdir(d) if x.endswith('.py')]
	e = d + os.sep + 'Tools'
	lst.extend([e + os.sep + x for x in os.listdir(e) if x.endswith('.py')])
	f = d + os.sep + 'extras'
	lst.extend([f + os.sep + x for x in os.listdir(f) if x.endswith('.py')])

	random.shuffle(lst)
	#lst.sort()
	return lst

def gen(lst, options):

	if options.maxi:
		opti_ref = 0
		filename = 'max.tar.%s' % options.kind
		def compare(a, b):
			return a > b
	else:
		opti_ref = 1000000000
		filename = 'min.tar.%s' % options.kind
		def compare(a, b):
			return a < b
	cmd = 'tar %s %s ' % (comp[options.kind], filename)
	opti = [opti_ref]

	LEN = len(lst)

	POP = 3*LEN + 1
	popul = [range(LEN) for x in range(POP)]
	fitn = [0 for x in range(POP)]

	def rnd():
		return random.randint(0, LEN -1)

	def mutate():
		for x in range(LEN):
			# rotate the previous element by one
			v = popul[x+LEN] = popul[x+LEN - 1]
			a = v.pop(0)
			v.append(a)

		for x in range(LEN):
			# swap elements
			a = rnd()
			b = rnd()

			v = popul[x]
			c = v[a]
			v[a] = v[b]
			v[b] = c

		for x in range(LEN):
			# get one element out, add at the end
			v = popul[x+2*LEN]

			a = rnd()
			c = v[a]
			del v[a]
			v.append(c)

	def evil():

		best = opti_ref
		pos = -1
		for x in range(len(popul)):
			v = popul[x]
			arr = [lst[a] for a in v]
			tmp = '%s %s' % (cmd, ' '.join(arr))
			subprocess.Popen(tmp, shell=True).wait()
			siz = os.stat(filename).st_size

			fitn[x] = siz
			if compare(siz, best):
				best = siz
				pos = x

				if compare(siz, opti[0]):
					opti[0] = siz
					shutil.copy2(filename, 'best_' + filename)

			#print popul[x], sum(popul[x]), sum(range(LEN))
			assert (sum(popul[x]) == sum(range(LEN)))

		#print pos
		for x in range(len(popul)):
			if x == pos:
				continue
			popul[x] = popul[pos][:]
			assert(len(popul[x]) == LEN)
		return best

	for i in range(10000):
		mutate()
		print(evil())

if __name__ == '__main__':

	parser = optparse.OptionParser()
	parser.add_option('--max', dest='maxi', default=False, action='store_true', help='maximize the file size (default is minimize)')
	parser.add_option('--kind', dest='kind', default='bz2', action='store', help='bz2, xz or gz')
	(options, args) = parser.parse_args()

	lst = read_wafdir()
	gen(lst, options)

