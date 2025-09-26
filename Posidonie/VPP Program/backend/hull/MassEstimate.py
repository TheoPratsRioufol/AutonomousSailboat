

# surfaces in m^2

top_surface = 0.99
side_surface = (0.16 + 0.53)*2
back_surface = 0.06
middle_surface = 2*0.23

PHY_RHO_CTP = 510
PHY_RHO_FIBERGLASS = 2540 # With polyester glue in it
E_SKIN_FIBERGLASS_EXT = 2.5*1e-3 # width of the fiberglass skin [m]
E_SKIN_FIBERGLASS_INT = 1.5*1e-3 # width of the fiberglass skin [m]

def getMass(S, eWood):
    return S*eWood*PHY_RHO_CTP + S*(E_SKIN_FIBERGLASS_EXT + E_SKIN_FIBERGLASS_INT)*PHY_RHO_FIBERGLASS

mass = getMass(top_surface + back_surface, 15*1e-3) + getMass(side_surface, 5*1e-3)
# estimates of the couples
mass += 4*getMass(back_surface, 10*1e-3) + getMass(middle_surface, 10*1e-3)

print("Total mass=", "{:.2f}".format(mass),"kg")
