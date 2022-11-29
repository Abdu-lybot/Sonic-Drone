import rospy
import sqlite3
from threading import Lock
#from self_exceptions import InvalidQueryDatabase


class DatabaseOperations:
    def __init__(self):
        self.db = rospy.get_param('database_robot/database/path')
        self.lock = Lock()

    def _execute_query(self, query, values):
        '''
        Executes a query in the database
        :param query: query to execute
        :param values: values to repace in the query given as a tuple or list of tuples with the appropriate type
        :return:
        '''
        self.lock.acquire()
        conn = sqlite3.connect(self.db)
        c = conn.cursor()
        try:
            c.execute('PRAGMA foreign_keys = ON')
            if type(values) == list:
                c.executemany(query, values)
            else:
                c.execute(query, values)
            conn.commit()
            return c.fetchall()
        except Exception as e:
            message = "[database_controller]: Exception when executing query: " + query + ". Error: " + e.message
            rospy.logerr(message)
            raise InvalidQueryDatabase(message)
        finally:
            self.lock.release()

    def insert(self, table, values, order):
        '''
        Inserts many values in the DB
        :param table: table to insert
        :param values: dict with the values to insert
        :param order: order of the fields to insert
        :return:
        '''
        #Create query
        spots = ["?" for i in range(0, len(order))]
        query = "INSERT INTO " + table + " VALUES " + "(" + ",".join(spots) + ")"
        #Transform values
        value_list = list()
        for value in values:
            value_tuple = tuple()
            for name in order:
                #FIXME: bad checking should be name in value
                if name in value:
                    value_tuple = value_tuple + (value[name], )
                else:
                    value_tuple = value_tuple + (None,)
            value_list.append(value_tuple)
        rospy.logdebug("[database_controller]: INSERT, execute query %s, with values %s", query, str(value_list))
        self._execute_query(query, value_list)
        return None

    #FIXME: Do it for more than one element
    def update(self, table, values, update_fields):
        '''
        Updates some values in the DB
        :param table: table to insert
        :param values: dict with the values to update
        :param update_fields: fields to update
        :return:
        '''
        #Separate between fields to update and to filter
        update_values = {key: values[0][key] if key in update_fields else None for key in values[0]}
        where_values = {key: values[0][key] if key not in update_fields else None for key in values[0]}
        #Clean up the dictionaries
        unwanted_update = set(update_values) - set(update_fields)
        for unwanted_key in unwanted_update:
            del update_values[unwanted_key]
        for unwanted_key in update_fields:
            del where_values[unwanted_key]
        #Create query
        query = "UPDATE " + table + " SET "
        #Transform update values
        value_tuple = tuple()
        # Check that the created dictionaries are not empty
        if bool(update_values) and bool(where_values):
            #Create the SET part of the query
            more_keys = ""
            for update_key in update_values:
                query = query + more_keys + update_key + "=?"
                more_keys = ", "
                value_tuple = value_tuple + (update_values[update_key],)
            #Create the WHERE part of the query
            query = query + " WHERE "
            more_keys = ""
            for where_key in where_values:
                query = query + more_keys + where_key + "=?"
                more_keys = " AND "
                value_tuple = value_tuple + (where_values[where_key],)
        rospy.logdebug("[database_controller]: UPDATE, execute query %s, with values %s", query, str(value_tuple))
        self._execute_query(query, value_tuple)
        return None

    #FIXME:Do it for more than one element
    def delete(self, table, values_list):
        query = "DELETE FROM " + table
        value_tuple = tuple()
        #Check if the list is empty (no parameters)
        if values_list:
            # DELETE only allows one element in the list
            values = values_list[0]
            query = query + " WHERE "
            ands = ""
            for value in values:
                query = query + ands + value + "=?"
                value_tuple = value_tuple + (values[value], )
                ands = " AND "
        rospy.logdebug("[database_controller]: DELETE, execute query %s, with values %s", query, str(value_tuple))
        self._execute_query(query, value_tuple)
        return None

    def _basic_select(self, table, values_list, select_fields, distinct=False):
        #Create Select
        if distinct:
            query = "SELECT DISTINCT "
        else:
            query = "SELECT "
        comma = ""
        if not select_fields:
            query = query + "*"
        else:
            for field in select_fields:
                query = query + comma + field
                comma = ", "
        #Create From
        query = query + " FROM " + table
        #Create Where
        value_tuple = tuple()
        #Check if the list is empty (no parameters)
        if values_list:
            # SELECT only allows one element in the list
            values = values_list[0]
            query = query + " WHERE "
            ands = ""
            for value in values:
                query = query + ands + value + "=?"
                value_tuple = value_tuple + (values[value],)
                ands = " AND "
        rospy.logdebug("[database_controller]: SELECT, execute query %s, with values %s", query, str(value_tuple))
        data = self._execute_query(query, value_tuple)
        rospy.logdebug("[database_controller]: query executed with obtained data %s", str(data))
        return data

    #FIXME: Do it for more than one element
    def select(self, table, values_list, select_fields):
        return self._basic_select(table, values_list, select_fields)

    #FIXME: Do it for more than one element
    def select_distinct(self, table, values_list, select_fields):
        return self._basic_select(table, values_list, select_fields, True)
