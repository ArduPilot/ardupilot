from rstemit import RSTEmit


class RSTLATEXPDFEmit(RSTEmit):
    def __init__(self, *args, **kwargs):
        RSTEmit.__init__(self, *args, **kwargs)

    def output_fname(self):
        return 'ParametersLatex.rst'

    def render_table_headings(self, ret, row, headings, field_table_info, field, param):
        # add to ret rather than append rows
        ret = ''
        if field in ['Values', 'Bitmask']:
            ret = "\n\n" + self.render_prog_values_field(field_table_info[field], param, field) + "\n\n"
        else:
            row.append(self.render_prog_values_field(field_table_info[field], param, field))

        # remove Values and Bitmask items from headings list
        while any(x in headings for x in ['Values', 'Bitmask']):
            try:
                headings.remove('Bitmask')
            except ValueError:
                pass
            try:
                headings.remove('Values')
            except ValueError:
                pass
        return ret
