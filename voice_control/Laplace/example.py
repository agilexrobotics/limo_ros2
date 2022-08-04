import os
import laplace
ll = laplace.Laplace(Plugin_path=os.path.dirname(__file__)+"/examples/Plugins")
ll.spin()