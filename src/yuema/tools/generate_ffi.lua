require('cparser')

local input, output, prefix = ...
if not input or not output or not prefix then
	error("Usage: generate_ffi <infile> <outfile> <prefix> [<defines>]")
end

local parseOptions = {
	silent = true
}

do
	local defs = select(4, ...)
	if defs then
		for def in defs:gmatch('%S+') do
			table.insert(parseOptions, ('-D%s'):format(def))
		end
	end
end

local FFICODE = [[
typedef struct {
	%s
} %s_API;

%s_API %s_EXPORTS = {
	%s
};

const char %s_FFI[] = R"ffiexports(
%s
%s
typedef struct {
	%s
} %s_API;
)ffiexports";
]]

local function typeIs(ty,tag)
	assert(ty)
	if ty.tag == 'Qualified' then ty = ty.t end
	return ty.tag == tag
end

local function declToString(action)
	local tag = action and action.tag
	if tag == 'TypeDef' or tag == 'Definition' or tag == 'Declaration' then
		local n = (action.sclass == '[typetag]') and "" or action.name
		local s = cparser.typeToString(action.type, n)
		if tag == 'TypeDef' then
			s = 'typedef ' .. s
		end
		if action.intval then
			if action.sclass ~= '[enum]' then
				s = s .. ' = ' .. action.intval
			else
				s = ''
			end
		elseif action.init and typeIs(action.type, 'Function') then
			error(s)
		end
		return s
	end
end

local function getConstants()
	local result = {}
	local li = cparser.declarationIterator(parseOptions, io.lines(input), input)
	for action in li do
		if action.tag == 'CppEvent' and action.directive == 'define' then
			if action.intval then
				table.insert(result, ('static const int %s = %s;'):format(action.name, action.intval))
			else
				print(("WARNING: can't generate ffi code for #define %s"):format(action.name))
			end
		end
	end
	return result;
end

local function getDeclarations()
	local result = { types = {}, functions = {} }
	local li = cparser.declarationIterator(parseOptions, io.lines(input), input)
	for action in li do
		if action.type then
			local decl = declToString(action)
			if decl and #decl > 0 then
				if action.type.tag == 'Function' then
					table.insert(result.functions, ('%s;'):format(decl))
				else
					table.insert(result.types, ('%s;'):format(decl))
				end
			end
		end
	end
	return result
end

local function getFunctions()
	local result = { types = {}, names = {} }
	local li = cparser.declarationIterator(parseOptions, io.lines(input), input)
	for action in li do
		if action.tag == 'Declaration' and action.type.tag == 'Function' then
			local f = action.type
			local args = {}
			for _, pair in ipairs(f) do
				if pair.ellipsis then
					table.insert(args, '...')
				else
					table.insert(args, cparser.typeToString(pair[1], pair[2]))
				end
			end

			local returnType = cparser.typeToString(action.type.t, '')
			local funcName = action.name
			local argList = table.concat(args, ', ')

			local ptrType = ('%s (*%s)(%s);'):format(returnType, funcName, argList)
			table.insert(result.types, ptrType)
			table.insert(result.names, funcName)
		end
	end
	return result
end

local function generateCode(decls, decls, funcs, consts)
	local code = FFICODE:format(
		table.concat(funcs.types, '\n\t'),
		prefix,
		prefix,
		prefix,
		table.concat(funcs.names, ',\n\t'),

		prefix,
		table.concat(decls.types, '\n'),
		table.concat(consts, '\n'),
		table.concat(funcs.types, '\n\t'),
		prefix
	)

	local file = io.open(output, 'w')
	file:write(code)
	file:close()
end

local decls = getDeclarations()
local funcs = getFunctions()
local consts = getConstants()
generateCode(decls, decls, funcs, consts)
