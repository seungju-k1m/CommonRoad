from crdesigner.conversion.sumo_map.sumo2cr import convert_net_to_cr

net_file = './test/helloworld.net.xml'
output_folder = './test'
cr_mat_path = convert_net_to_cr(net_file, output_folder)
