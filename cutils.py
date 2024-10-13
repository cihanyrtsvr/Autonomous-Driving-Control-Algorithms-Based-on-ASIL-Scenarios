"""Utils function for creating MPC class variable"""


class CUtils:
    """Utility function class

    Args:
        object (_type_): depend on type creates variable in the class
    """

    def __init__(self):
        pass

    def create_var(self, var_name, value):
        """Creates the MPC variables

        Args:
            var_name (string): name of creating variable
            value (any): the value that be
        """
        if not var_name in self.__dict__:
            self.__dict__[var_name] = value

    def update_var(self, var_name, new_value):
        """Updates the value of an existing variable

        Args:
            var_name (string): name of the variable to be updated
            new_value (any): the new value to assign to the variable
        """
        if var_name in self.__dict__:
            self.__dict__[var_name] = new_value

    def delete_var(self, var_name):
        """Deletes a variable from the class

        Args:
            var_name (string): name of the variable to be deleted
        """
        if var_name in self.__dict__:
            del self.__dict__[var_name]
