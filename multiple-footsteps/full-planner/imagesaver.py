# Generate the image to be saved

import sys
import os 
import string
import random
def id_gen(dim, size=6, chars=string.ascii_uppercase+string.digits):
	dir_path = os.path.dirname(os.path.realpath(__file__))
	results_dir = os.path.join(dir_path, 'images/')
	file_name = str(dim) + 'D-' + ''.join(random.choice(chars) for _ in range(size)) + '.png'
	if not os.path.isdir(results_dir):
		os.makedirs(results_dir)
	return results_dir + file_name
	
