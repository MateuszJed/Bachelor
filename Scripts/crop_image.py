# Importing Image class from PIL module
from PIL import Image
import glob
path = r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Bachelor\Bachelor\Bilder\New result\Module-y-v2.png"
# Opens a image in RGB mode
# for path in glob.iglob(pathh, recursive=True):
    
im = Image.open(path)

# Size of the image in pixels (size of original image)
# (This is not mandatory)
width, height = im.size

# Setting the points for cropped image
left = 100
top = 0
right = width-left
bottom = height

# Cropped image of above dimension
# (It will not change original image)
im1 = im.crop((left, top, right, bottom))

# Shows the image in image viewer
# im1.show()
im1 = im1.save(path)