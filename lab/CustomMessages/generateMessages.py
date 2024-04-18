from os import mkdir
from os.path import exists, dirname, join
import jinja2
from textx import metamodel_from_file

this_folder = dirname(__file__)
print(this_folder)

def main(debug=False):

    # Instantiate the custom message mm
    mmCustomMessage = metamodel_from_file(join(this_folder,'metamodel/CustomMessageGrammar.tx'))



    # Create the output folder for the custom messages
    srcgen_folder = join(this_folder, 'Generated')
    if not exists(srcgen_folder):
        mkdir(srcgen_folder)

    # Initialize the Templates engine.
    jinja_env = jinja2.Environment(loader=jinja2.FileSystemLoader(this_folder),trim_blocks=True,lstrip_blocks=True)

    # Load the html Templates
    template = jinja_env.get_template('Templates/pythonMessage.template')

    # Fetch the custom message
    PathToMessage = 'examples/example1.msg'
    customMessage = mmCustomMessage.model_from_file(join(this_folder,PathToMessage))

    # determine file name
    name = PathToMessage.split("/")[-1].split(".msg")[0]

    # Generate python code
    with open(join(srcgen_folder,"Messages_GEN.py"), 'w') as f:
        f.write(template.render(name=name,customMessage=customMessage,mmCustomMessage=mmCustomMessage))



if __name__ == "__main__":
    main()