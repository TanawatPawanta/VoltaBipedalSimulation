"""Reference walking-velocity commands v_ref(t) = [vx, vy, omega] used by the experiments."""


def GetVelocityProfile(profile_type:str,vel:float=0.1):
    '''
    Returns a velocity profile function based on the specified type.
    Args:
        profile_type (str): The type of velocity profile. Options include 'straight', 'diagonal', 'circular', 'rotational', and 'combined'.
        vel (float): The velocity for the profile.
    Returns:
        function: A function that takes time (t) as input and returns the corresponding velocity command as a list [vx, vy, omega].
    '''
    # print("recieve cmd:", profile_type)
    match profile_type:
        case 'forward_backward':
            def VRefCommand(t):
                if t < 2.0:
                    return [0.1, 0.0, 0.0]
                elif t<5.0:
                    return [-0.1, 0.0, 0.0]
                else:
                    return [0.0, 0.0, 0.0]
        case 'straight':
            def VRefCommand(t):
                if t < 4.0:
                    return [0.06, 0.0, 0.0]
                elif t<10.0:
                    return [0.12, 0.0, 0.0]
                else:
                    return [0.0, 0.0, 0.0]
        case 'straight_10':
            def VRefCommand(t):
                if t < 10.0:
                    return [vel, 0.0, 0.0]
                else:
                    return [0.0, 0.0, 0.0]
        case 'straight_20':
            def VRefCommand(t):
                if t < 20.0:
                    return [vel, 0.0, 0.0]
                else:
                    return [0.0, 0.0, 0.0]
        case 'diagonal':
            def VRefCommand(t):
                if t<10.0:
                    return [0.05, 0.1, 0.0]
                else:
                    return [0.00, 0.0, 0.0]
        case 'circular':
            def VRefCommand(t):
                if t<10.0:
                    return [0.05, 0.0,0.1]
                else:
                    return [0.00, 0.0, 0.0]
        case 'rotational':
            def VRefCommand(t):
                if t<10.0:
                    return [0.0, 0.0,0.1]
                else:
                    return [0.00, 0.0, 0.0]
        case 'L':
            def VRefCommand(t):
                if t<5.0:
                    return [0.1, 0.0,0.0]
                elif t<10.0:
                    return [0.0, 0.1, 0.0]
                else:
                    return [0.0, 0.0, 0.0]
        case 'combined':
            def VRefCommand(t):
                if t < 2.0:
                    return [0.1, 0.0, 0.0]
                elif t < 7.0:
                    return [0.1, 0.0, -0.15]
                elif t < 12.0:
                    return [0.1, 0.0, 0.15]
                elif t < 16.0:
                    return [0.05, 0.06, 0.0]
                elif t < 20.0:
                    return [0.08, 0.0, 0.0]
                else:
                    return [0.0, 0.0, 0.0]

        case "0":
            def VRefCommand(t):
                return [0.0, 0.0, 0.0]

        case _:
            raise ValueError(f"Unknown profile type: {profile_type}")
    return VRefCommand
