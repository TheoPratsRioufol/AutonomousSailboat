from stl import mesh
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from PIL import Image, ImageChops

def fig2img(fig):
    """Convert a Matplotlib figure to a PIL Image and return it"""
    import io
    buf = io.BytesIO()
    fig.savefig(buf, dpi=300)
    buf.seek(0)
    img = Image.open(buf)
    return img

def crop_to_content(image):
    gray = image.convert("L")
    bg = Image.new("L", gray.size, 255)
    diff = ImageChops.difference(gray, bg)
    bbox = diff.getbbox()
    if bbox:
        return image.crop(bbox)
    return image

def getImgFromSTL(path, color='red'):
    print("Loading from:",path)

    fig = plt.figure()
    axes = fig.add_subplot(projection='3d')

    # Load the file
    boat = mesh.Mesh.from_file(path)
    axes.add_collection3d(mplot3d.art3d.Poly3DCollection(boat.vectors,
                      shade=True,
                      facecolors=color,
                      edgecolors=color))
    scale = boat.points.flatten()
    axes.auto_scale_xyz(scale, scale, scale)
    axes.view_init(elev=45, azim=-45, roll=0)
    axes.set_axis_off()
    img = fig2img(fig)
    return crop_to_content(img)

if __name__ == "__main__":
    img = getImgFromSTL('export/goodOrientation.stl')
    img.show()