# -*- coding: utf-8 -*-

from itertools import product
from re import findall
import httplib2
import apiclient
from oauth2client import client
from oauth2client import tools
from oauth2client.service_account import ServiceAccountCredentials


## @brief Download one sheet from Google Sheets with given credentials
def download_sheet(creds_file, spreadsheet_id, range_name, render_option='FORMULA'):
    def get_values(service, spreadsheet_id, range_name):
        try:
            result = service.spreadsheets().values().get(
                spreadsheetId=spreadsheet_id,
                range=range_name,
                valueRenderOption=render_option # FORMATTED_VALUE, UNFORMATTED_VALUE, FORMULA
                ).execute()
        except apiclient.errors.HttpError: # no such sheet
            sheet_metadata = service.spreadsheets().get(spreadsheetId=spreadsheet_id).execute()
            sheets = sheet_metadata.get('sheets', '')
            period_prefix, period_num = range_name.split('!')[0].rsplit(' ', 1)
            period_num = int(period_num)
            titles = list()
            for s in sheets:
                t = s.get("properties", {}).get("title", "")
                if period_prefix in t:
                    titles.append(t)
            avail_period_num = len(titles)
            range_name = period_prefix + ' ' + str(period_num % avail_period_num) # ring periods
            result = service.spreadsheets().values().get(
                spreadsheetId=spreadsheet_id,
                range=range_name,
                valueRenderOption=render_option # FORMATTED_VALUE, UNFORMATTED_VALUE, FORMULA
                ).execute()            
        return result.get('values')

    def get_service(creds_file):
        credentials = ServiceAccountCredentials.from_json_keyfile_name(creds_file,
                                ['https://www.googleapis.com/auth/spreadsheets',
                                'https://www.googleapis.com/auth/drive'])
        httpAuth = credentials.authorize(httplib2.Http())
        service = apiclient.discovery.build('sheets', 'v4', http = httpAuth)
        http = credentials.authorize(httplib2.Http())

        discoveryUrl = ('https://sheets.googleapis.com/$discovery/rest?'
                        'version=v4')
        service = apiclient.discovery.build('sheets', 'v4', http=http,
                                            discoveryServiceUrl=discoveryUrl)
        return service
    
    content = get_values(get_service(creds_file), spreadsheet_id, range_name)
    return content


## @brief Cut mess blelow and above the chart in sheet
def extract_chart(content, topleft_content='Price', cols_num=152):
    row_chart = 0
    for row_number, row_content in enumerate(content):
        if topleft_content in row_content:
            row_chart = row_number
            break
    chart = content[row_chart:row_chart + cols_num]
    return chart


def download_chart(creds_file, spreadsheet_id, range_name, cols_num=152):
    content = download_sheet(creds_file, spreadsheet_id, range_name,
                             render_option='UNFORMATTED_VALUE')
    chart = extract_chart(content, cols_num=cols_num)
    return chart


## @brief Get lot parameters from chart
## @param max_price_line expected format:
##   ask:
##    max_price_line[0] - max price, int
##    max_price_line[1] - fee, int
##    max_price_line[2] - yellow chunks ask, formula
##    max_price_line[7] - green chunks ask, formula
##    max_price_line[12] - blue chunks ask, formula
##    max_price_line[17] - purple chunks ask, formula
## @return dict with parameters for robonomics_market/Asks(Bids)Generator message
def extract_lots_params(max_price_line, lots_type):
    ## @brief Get 'a' and 'k' params of bid/ask law in format like '= - 2.1 +0,05 * $A41'
    def extract_ak(formula):
        nums = findall( r"[+-]?\d+(?:\.\d+)?", formula.replace(',', '.').replace(' ', '') )
        if len(nums) == 3: # get first two as a and k
            a = nums[0]
            k = nums[1]
        elif len(nums) == 2: # get first as k
            a = 0
            k = nums[0]
        elif len(nums) == 1: # get first as a
            a = nums[0]
            k = 0
        return {'a': float(a), 'k': float(k)}

    lots = dict()
    price_range = max_price_line[0]
    fee = max_price_line[1]
    colors = ['yellow', 'green', 'blue', 'purple'] # rainbow order like in the sheet
    if lots_type == 'ask':
        for i, c in enumerate(colors):
            ak = extract_ak( max_price_line[2 + 5*i] ) # each 5th column after 2nd
            ak['k'] = -ak['k']
            lots[c] = ak
            lots[c].update({'fee': fee, 'price_range': price_range})
    elif lots_type == 'bid':
        factories = ['welder', 'driller', 'miller', 'cnc']
        for i, f in enumerate(factories):
            # for each fact bids for each color are in each 5th col after 3rd + f.num*5
            lots[f] = {c : extract_ak( max_price_line[3 + i + 5*j] )
                                       for j, c in enumerate(colors)}
            for lot in lots[f].values():
                lot.update({'fee': fee, 'price_range': price_range})
    return lots


## @brief Download asks parameters from given spreadsheet
def asks_params(creds_file, spreadsheet_id, range_name):
    # print("Getting asks: %s, %s, %s" % (creds_file, spreadsheet_id, range_name))
    content = download_sheet(creds_file, spreadsheet_id, range_name)
    chart = extract_chart(content)
    print(chart)
    print(extract_lots_params(chart[-1], 'ask'))
    return extract_lots_params(chart[-1], 'ask')


## @brief Download bids parameters from given spreadsheet
def bids_params(creds_file, spreadsheet_id, range_name):
    # print("Getting bids: %s, %s, %s" % (creds_file, spreadsheet_id, range_name))
    content = download_sheet(creds_file, spreadsheet_id, range_name)
    chart = extract_chart(content)
    print(chart)
    print(extract_lots_params(chart[-1], 'bid'))
    return extract_lots_params(chart[-1], 'bid')


def chart_to_dict(chart):
    colors = ['yellow', 'green', 'blue', 'purple'] # rainbow order like in the sheet
    factories = ['welder', 'driller', 'miller', 'cnc'] # order like in the spreadsheet
    dchart = list()
    for i in range(2, len(chart)): # from non-zero price to last row
        dchart.append(
        {'Price': chart[i][0],
         'Fee': chart[i][1],
         'Lots': {c: {'ask': chart[i][2 + 5*ci],
                      'bids': {f:chart[i][3 + fi + 5*ci] for fi, f in enumerate(factories)}
                     } for ci, c in enumerate(colors)
                 }
        })
    return dchart


def get_matched_asks(dchart):
    colors = ['yellow', 'green', 'blue', 'purple'] # rainbow order like in the sheet
    factories = ['welder', 'driller', 'miller', 'cnc']
    matched_asks = list()
    for row in dchart:
        p = row['Price']
        fee = row['Fee']
        lots = row['Lots']
        for c in colors:
            ask_quantity = lots[c]['ask']
            if ask_quantity < 0 or ask_quantity % 1 != 0: # drop non natural quantity
                continue
            matches = 0
            for f in factories:
                bid_quantity = lots[c]['bids'][f]
                if ask_quantity == bid_quantity:
                    matches += 1
            q = ask_quantity
            m = matches
            matched_asks.append({'Color': c, 'Price': p, 'Fee': fee, 'Quantity': q, 'Matches': m})
    return matched_asks

def get_matched_bids(dchart):
    colors = ['yellow', 'green', 'blue', 'purple'] # rainbow order like in the sheet
    factories = ['welder', 'driller', 'miller', 'cnc']
    bids = list()
    for row in dchart:
        p = row['Price']
        fee = row['Fee']
        lots = row['Lots']
        for c in colors:
            ask_quantity = lots[c]['ask']
            if ask_quantity < 0 or ask_quantity % 1 != 0: # drop non natural quantity
                continue
            for f in factories:
                bid_quantity = lots[c]['bids'][f]
                if ask_quantity == bid_quantity:
                    q = ask_quantity
                    bids.append({'Factory': f, 'Color': c, 'Price': p, 'Fee': fee, 'Quantity': q})
    return bids


def download_matched_asks(creds_file, spreadsheet_id, range_name, cols_num=152):
    chart = download_chart(creds_file, spreadsheet_id, range_name, cols_num=152)
    dchart = chart_to_dict(chart)
    asks = get_matched_asks(dchart)
    return asks


def download_matched_bids(creds_file, spreadsheet_id, range_name, cols_num=152):
    chart = download_chart(creds_file, spreadsheet_id, range_name, cols_num=152)
    dchart = chart_to_dict(chart)
    bids = get_matched_bids(dchart)
    return bids
