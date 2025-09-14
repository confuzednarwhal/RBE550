import env_handler as Enviornment
import entity

test = Enviornment.map()
hero1 = entity.hero()

map = test.generate_map()
huh = hero1.gen_path(map)
test.display_field(huh)



# print(huh)
