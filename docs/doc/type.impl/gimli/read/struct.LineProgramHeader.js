(function() {var type_impls = {
"gimli":[["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-Clone-for-LineProgramHeader%3CR,+Offset%3E\" class=\"impl\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1022\">source</a><a href=\"#impl-Clone-for-LineProgramHeader%3CR,+Offset%3E\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;R, Offset&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/clone/trait.Clone.html\" title=\"trait core::clone::Clone\">Clone</a> for <a class=\"struct\" href=\"gimli/read/struct.LineProgramHeader.html\" title=\"struct gimli::read::LineProgramHeader\">LineProgramHeader</a>&lt;R, Offset&gt;<div class=\"where\">where\n    R: <a class=\"trait\" href=\"gimli/read/trait.Reader.html\" title=\"trait gimli::read::Reader\">Reader</a>&lt;Offset = Offset&gt; + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/clone/trait.Clone.html\" title=\"trait core::clone::Clone\">Clone</a>,\n    Offset: <a class=\"trait\" href=\"gimli/read/trait.ReaderOffset.html\" title=\"trait gimli::read::ReaderOffset\">ReaderOffset</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/clone/trait.Clone.html\" title=\"trait core::clone::Clone\">Clone</a>,</div></h3></section></summary><div class=\"impl-items\"><details class=\"toggle method-toggle\" open><summary><section id=\"method.clone\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1022\">source</a><a href=\"#method.clone\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://doc.rust-lang.org/1.79.0/core/clone/trait.Clone.html#tymethod.clone\" class=\"fn\">clone</a>(&amp;self) -&gt; <a class=\"struct\" href=\"gimli/read/struct.LineProgramHeader.html\" title=\"struct gimli::read::LineProgramHeader\">LineProgramHeader</a>&lt;R, Offset&gt;</h4></section></summary><div class='docblock'>Returns a copy of the value. <a href=\"https://doc.rust-lang.org/1.79.0/core/clone/trait.Clone.html#tymethod.clone\">Read more</a></div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.clone_from\" class=\"method trait-impl\"><span class=\"rightside\"><span class=\"since\" title=\"Stable since Rust version 1.0.0\">1.0.0</span> · <a class=\"src\" href=\"https://doc.rust-lang.org/1.79.0/src/core/clone.rs.html#169\">source</a></span><a href=\"#method.clone_from\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://doc.rust-lang.org/1.79.0/core/clone/trait.Clone.html#method.clone_from\" class=\"fn\">clone_from</a>(&amp;mut self, source: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.reference.html\">&amp;Self</a>)</h4></section></summary><div class='docblock'>Performs copy-assignment from <code>source</code>. <a href=\"https://doc.rust-lang.org/1.79.0/core/clone/trait.Clone.html#method.clone_from\">Read more</a></div></details></div></details>","Clone","gimli::read::line::LineNumberProgramHeader"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-Debug-for-LineProgramHeader%3CR,+Offset%3E\" class=\"impl\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1022\">source</a><a href=\"#impl-Debug-for-LineProgramHeader%3CR,+Offset%3E\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;R, Offset&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/fmt/trait.Debug.html\" title=\"trait core::fmt::Debug\">Debug</a> for <a class=\"struct\" href=\"gimli/read/struct.LineProgramHeader.html\" title=\"struct gimli::read::LineProgramHeader\">LineProgramHeader</a>&lt;R, Offset&gt;<div class=\"where\">where\n    R: <a class=\"trait\" href=\"gimli/read/trait.Reader.html\" title=\"trait gimli::read::Reader\">Reader</a>&lt;Offset = Offset&gt; + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/fmt/trait.Debug.html\" title=\"trait core::fmt::Debug\">Debug</a>,\n    Offset: <a class=\"trait\" href=\"gimli/read/trait.ReaderOffset.html\" title=\"trait gimli::read::ReaderOffset\">ReaderOffset</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/fmt/trait.Debug.html\" title=\"trait core::fmt::Debug\">Debug</a>,</div></h3></section></summary><div class=\"impl-items\"><details class=\"toggle method-toggle\" open><summary><section id=\"method.fmt\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1022\">source</a><a href=\"#method.fmt\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://doc.rust-lang.org/1.79.0/core/fmt/trait.Debug.html#tymethod.fmt\" class=\"fn\">fmt</a>(&amp;self, f: &amp;mut <a class=\"struct\" href=\"https://doc.rust-lang.org/1.79.0/core/fmt/struct.Formatter.html\" title=\"struct core::fmt::Formatter\">Formatter</a>&lt;'_&gt;) -&gt; <a class=\"type\" href=\"https://doc.rust-lang.org/1.79.0/core/fmt/type.Result.html\" title=\"type core::fmt::Result\">Result</a></h4></section></summary><div class='docblock'>Formats the value using the given formatter. <a href=\"https://doc.rust-lang.org/1.79.0/core/fmt/trait.Debug.html#tymethod.fmt\">Read more</a></div></details></div></details>","Debug","gimli::read::line::LineNumberProgramHeader"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-LineProgramHeader%3CR,+Offset%3E\" class=\"impl\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1076-1420\">source</a><a href=\"#impl-LineProgramHeader%3CR,+Offset%3E\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;R, Offset&gt; <a class=\"struct\" href=\"gimli/read/struct.LineProgramHeader.html\" title=\"struct gimli::read::LineProgramHeader\">LineProgramHeader</a>&lt;R, Offset&gt;<div class=\"where\">where\n    R: <a class=\"trait\" href=\"gimli/read/trait.Reader.html\" title=\"trait gimli::read::Reader\">Reader</a>&lt;Offset = Offset&gt;,\n    Offset: <a class=\"trait\" href=\"gimli/read/trait.ReaderOffset.html\" title=\"trait gimli::read::ReaderOffset\">ReaderOffset</a>,</div></h3></section></summary><div class=\"impl-items\"><details class=\"toggle method-toggle\" open><summary><section id=\"method.offset\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1082-1084\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.offset\" class=\"fn\">offset</a>(&amp;self) -&gt; <a class=\"struct\" href=\"gimli/struct.DebugLineOffset.html\" title=\"struct gimli::DebugLineOffset\">DebugLineOffset</a>&lt;R::<a class=\"associatedtype\" href=\"gimli/read/trait.Reader.html#associatedtype.Offset\" title=\"type gimli::read::Reader::Offset\">Offset</a>&gt;</h4></section></summary><div class=\"docblock\"><p>Return the offset of the line number program header in the <code>.debug_line</code> section.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.unit_length\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1088-1090\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.unit_length\" class=\"fn\">unit_length</a>(&amp;self) -&gt; R::<a class=\"associatedtype\" href=\"gimli/read/trait.Reader.html#associatedtype.Offset\" title=\"type gimli::read::Reader::Offset\">Offset</a></h4></section></summary><div class=\"docblock\"><p>Return the length of the line number program and header, not including\nthe length of the encoded length itself.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.encoding\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1093-1095\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.encoding\" class=\"fn\">encoding</a>(&amp;self) -&gt; <a class=\"struct\" href=\"gimli/struct.Encoding.html\" title=\"struct gimli::Encoding\">Encoding</a></h4></section></summary><div class=\"docblock\"><p>Return the encoding parameters for this header’s line program.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.version\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1098-1100\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.version\" class=\"fn\">version</a>(&amp;self) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.u16.html\">u16</a></h4></section></summary><div class=\"docblock\"><p>Get the version of this header’s line program.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.header_length\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1104-1106\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.header_length\" class=\"fn\">header_length</a>(&amp;self) -&gt; R::<a class=\"associatedtype\" href=\"gimli/read/trait.Reader.html#associatedtype.Offset\" title=\"type gimli::read::Reader::Offset\">Offset</a></h4></section></summary><div class=\"docblock\"><p>Get the length of the encoded line number program header, not including\nthe length of the encoded length itself.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.address_size\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1109-1111\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.address_size\" class=\"fn\">address_size</a>(&amp;self) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.u8.html\">u8</a></h4></section></summary><div class=\"docblock\"><p>Get the size in bytes of a target machine address.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.format\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1114-1116\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.format\" class=\"fn\">format</a>(&amp;self) -&gt; <a class=\"enum\" href=\"gimli/enum.Format.html\" title=\"enum gimli::Format\">Format</a></h4></section></summary><div class=\"docblock\"><p>Whether this line program is encoded in 64- or 32-bit DWARF.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.line_encoding\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1119-1121\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.line_encoding\" class=\"fn\">line_encoding</a>(&amp;self) -&gt; <a class=\"struct\" href=\"gimli/struct.LineEncoding.html\" title=\"struct gimli::LineEncoding\">LineEncoding</a></h4></section></summary><div class=\"docblock\"><p>Get the line encoding parameters for this header’s line program.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.minimum_instruction_length\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1125-1127\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.minimum_instruction_length\" class=\"fn\">minimum_instruction_length</a>(&amp;self) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.u8.html\">u8</a></h4></section></summary><div class=\"docblock\"><p>Get the minimum instruction length any instruction in this header’s line\nprogram may have.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.maximum_operations_per_instruction\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1131-1133\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.maximum_operations_per_instruction\" class=\"fn\">maximum_operations_per_instruction</a>(&amp;self) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.u8.html\">u8</a></h4></section></summary><div class=\"docblock\"><p>Get the maximum number of operations each instruction in this header’s\nline program may have.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.default_is_stmt\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1137-1139\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.default_is_stmt\" class=\"fn\">default_is_stmt</a>(&amp;self) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.bool.html\">bool</a></h4></section></summary><div class=\"docblock\"><p>Get the default value of the <code>is_stmt</code> register for this header’s line\nprogram.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.line_base\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1142-1144\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.line_base\" class=\"fn\">line_base</a>(&amp;self) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.i8.html\">i8</a></h4></section></summary><div class=\"docblock\"><p>Get the line base for this header’s line program.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.line_range\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1147-1149\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.line_range\" class=\"fn\">line_range</a>(&amp;self) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.u8.html\">u8</a></h4></section></summary><div class=\"docblock\"><p>Get the line range for this header’s line program.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.opcode_base\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1152-1154\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.opcode_base\" class=\"fn\">opcode_base</a>(&amp;self) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.u8.html\">u8</a></h4></section></summary><div class=\"docblock\"><p>Get opcode base for this header’s line program.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.standard_opcode_lengths\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1158-1160\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.standard_opcode_lengths\" class=\"fn\">standard_opcode_lengths</a>(&amp;self) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.reference.html\">&amp;R</a></h4></section></summary><div class=\"docblock\"><p>An array of <code>u8</code> that specifies the number of LEB128 operands for\neach of the standard opcodes.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.directory_entry_format\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1163-1165\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.directory_entry_format\" class=\"fn\">directory_entry_format</a>(&amp;self) -&gt; &amp;[<a class=\"struct\" href=\"gimli/read/struct.FileEntryFormat.html\" title=\"struct gimli::read::FileEntryFormat\">FileEntryFormat</a>]</h4></section></summary><div class=\"docblock\"><p>Get the format of a directory entry.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.include_directories\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1171-1173\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.include_directories\" class=\"fn\">include_directories</a>(&amp;self) -&gt; &amp;[<a class=\"enum\" href=\"gimli/read/enum.AttributeValue.html\" title=\"enum gimli::read::AttributeValue\">AttributeValue</a>&lt;R, Offset&gt;]</h4></section></summary><div class=\"docblock\"><p>Get the set of include directories for this header’s line program.</p>\n<p>For DWARF version &lt;= 4, the compilation’s current directory is not included\nin the return value, but is implicitly considered to be in the set per spec.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.directory\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1178-1189\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.directory\" class=\"fn\">directory</a>(&amp;self, directory: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.u64.html\">u64</a>) -&gt; <a class=\"enum\" href=\"https://doc.rust-lang.org/1.79.0/core/option/enum.Option.html\" title=\"enum core::option::Option\">Option</a>&lt;<a class=\"enum\" href=\"gimli/read/enum.AttributeValue.html\" title=\"enum gimli::read::AttributeValue\">AttributeValue</a>&lt;R, Offset&gt;&gt;</h4></section></summary><div class=\"docblock\"><p>The include directory with the given directory index.</p>\n<p>A directory index of 0 corresponds to the compilation unit directory.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.file_name_entry_format\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1192-1194\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.file_name_entry_format\" class=\"fn\">file_name_entry_format</a>(&amp;self) -&gt; &amp;[<a class=\"struct\" href=\"gimli/read/struct.FileEntryFormat.html\" title=\"struct gimli::read::FileEntryFormat\">FileEntryFormat</a>]</h4></section></summary><div class=\"docblock\"><p>Get the format of a file name entry.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.file_has_timestamp\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1200-1206\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.file_has_timestamp\" class=\"fn\">file_has_timestamp</a>(&amp;self) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.bool.html\">bool</a></h4></section></summary><div class=\"docblock\"><p>Return true if the file entries may have valid timestamps.</p>\n<p>Only returns false if we definitely know that all timestamp fields\nare invalid.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.file_has_size\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1212-1218\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.file_has_size\" class=\"fn\">file_has_size</a>(&amp;self) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.bool.html\">bool</a></h4></section></summary><div class=\"docblock\"><p>Return true if the file entries may have valid sizes.</p>\n<p>Only returns false if we definitely know that all size fields\nare invalid.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.file_has_md5\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1221-1225\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.file_has_md5\" class=\"fn\">file_has_md5</a>(&amp;self) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.bool.html\">bool</a></h4></section></summary><div class=\"docblock\"><p>Return true if the file name entry format contains an MD5 field.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.file_names\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1228-1230\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.file_names\" class=\"fn\">file_names</a>(&amp;self) -&gt; &amp;[<a class=\"struct\" href=\"gimli/read/struct.FileEntry.html\" title=\"struct gimli::read::FileEntry\">FileEntry</a>&lt;R, Offset&gt;]</h4></section></summary><div class=\"docblock\"><p>Get the list of source files that appear in this header’s line program.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.file\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1237-1248\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.file\" class=\"fn\">file</a>(&amp;self, file: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.u64.html\">u64</a>) -&gt; <a class=\"enum\" href=\"https://doc.rust-lang.org/1.79.0/core/option/enum.Option.html\" title=\"enum core::option::Option\">Option</a>&lt;&amp;<a class=\"struct\" href=\"gimli/read/struct.FileEntry.html\" title=\"struct gimli::read::FileEntry\">FileEntry</a>&lt;R, Offset&gt;&gt;</h4></section></summary><div class=\"docblock\"><p>The source file with the given file index.</p>\n<p>A file index of 0 corresponds to the compilation unit file.\nNote that a file index of 0 is invalid for DWARF version &lt;= 4,\nbut we support it anyway.</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.raw_program_buf\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1268-1270\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.raw_program_buf\" class=\"fn\">raw_program_buf</a>(&amp;self) -&gt; R</h4></section></summary><div class=\"docblock\"><p>Get the raw, un-parsed <code>EndianSlice</code> containing this header’s line number\nprogram.</p>\n\n<div class=\"example-wrap\"><pre class=\"rust rust-example-rendered\"><code><span class=\"kw\">use </span>gimli::{LineProgramHeader, EndianSlice, NativeEndian};\n\n<span class=\"kw\">fn </span>get_line_number_program_header&lt;<span class=\"lifetime\">'a</span>&gt;() -&gt; LineProgramHeader&lt;EndianSlice&lt;<span class=\"lifetime\">'a</span>, NativeEndian&gt;&gt; {\n    <span class=\"comment\">// Get a line number program header from some offset in a\n    // `.debug_line` section...\n</span>}\n\n<span class=\"kw\">let </span>header = get_line_number_program_header();\n<span class=\"kw\">let </span>raw_program = header.raw_program_buf();\n<span class=\"macro\">println!</span>(<span class=\"string\">\"The length of the raw program in bytes is {}\"</span>, raw_program.len());</code></pre></div>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.instructions\" class=\"method\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1274-1278\">source</a><h4 class=\"code-header\">pub fn <a href=\"gimli/read/struct.LineProgramHeader.html#tymethod.instructions\" class=\"fn\">instructions</a>(&amp;self) -&gt; <a class=\"struct\" href=\"gimli/read/struct.LineInstructions.html\" title=\"struct gimli::read::LineInstructions\">LineInstructions</a>&lt;R&gt;</h4></section></summary><div class=\"docblock\"><p>Iterate over the instructions in this header’s line number program, parsing\nthem as we go.</p>\n</div></details></div></details>",0,"gimli::read::line::LineNumberProgramHeader"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-PartialEq-for-LineProgramHeader%3CR,+Offset%3E\" class=\"impl\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1022\">source</a><a href=\"#impl-PartialEq-for-LineProgramHeader%3CR,+Offset%3E\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;R, Offset&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a> for <a class=\"struct\" href=\"gimli/read/struct.LineProgramHeader.html\" title=\"struct gimli::read::LineProgramHeader\">LineProgramHeader</a>&lt;R, Offset&gt;<div class=\"where\">where\n    R: <a class=\"trait\" href=\"gimli/read/trait.Reader.html\" title=\"trait gimli::read::Reader\">Reader</a>&lt;Offset = Offset&gt; + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a>,\n    Offset: <a class=\"trait\" href=\"gimli/read/trait.ReaderOffset.html\" title=\"trait gimli::read::ReaderOffset\">ReaderOffset</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a>,</div></h3></section></summary><div class=\"impl-items\"><details class=\"toggle method-toggle\" open><summary><section id=\"method.eq\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1022\">source</a><a href=\"#method.eq\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://doc.rust-lang.org/1.79.0/core/cmp/trait.PartialEq.html#tymethod.eq\" class=\"fn\">eq</a>(&amp;self, other: &amp;<a class=\"struct\" href=\"gimli/read/struct.LineProgramHeader.html\" title=\"struct gimli::read::LineProgramHeader\">LineProgramHeader</a>&lt;R, Offset&gt;) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.bool.html\">bool</a></h4></section></summary><div class='docblock'>This method tests for <code>self</code> and <code>other</code> values to be equal, and is used\nby <code>==</code>.</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.ne\" class=\"method trait-impl\"><span class=\"rightside\"><span class=\"since\" title=\"Stable since Rust version 1.0.0\">1.0.0</span> · <a class=\"src\" href=\"https://doc.rust-lang.org/1.79.0/src/core/cmp.rs.html#263\">source</a></span><a href=\"#method.ne\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://doc.rust-lang.org/1.79.0/core/cmp/trait.PartialEq.html#method.ne\" class=\"fn\">ne</a>(&amp;self, other: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.reference.html\">&amp;Rhs</a>) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/core/primitive.bool.html\">bool</a></h4></section></summary><div class='docblock'>This method tests for <code>!=</code>. The default implementation is almost always\nsufficient, and should not be overridden without very good reason.</div></details></div></details>","PartialEq","gimli::read::line::LineNumberProgramHeader"],["<section id=\"impl-Eq-for-LineProgramHeader%3CR,+Offset%3E\" class=\"impl\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1022\">source</a><a href=\"#impl-Eq-for-LineProgramHeader%3CR,+Offset%3E\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;R, Offset&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/cmp/trait.Eq.html\" title=\"trait core::cmp::Eq\">Eq</a> for <a class=\"struct\" href=\"gimli/read/struct.LineProgramHeader.html\" title=\"struct gimli::read::LineProgramHeader\">LineProgramHeader</a>&lt;R, Offset&gt;<div class=\"where\">where\n    R: <a class=\"trait\" href=\"gimli/read/trait.Reader.html\" title=\"trait gimli::read::Reader\">Reader</a>&lt;Offset = Offset&gt; + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/cmp/trait.Eq.html\" title=\"trait core::cmp::Eq\">Eq</a>,\n    Offset: <a class=\"trait\" href=\"gimli/read/trait.ReaderOffset.html\" title=\"trait gimli::read::ReaderOffset\">ReaderOffset</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/cmp/trait.Eq.html\" title=\"trait core::cmp::Eq\">Eq</a>,</div></h3></section>","Eq","gimli::read::line::LineNumberProgramHeader"],["<section id=\"impl-StructuralPartialEq-for-LineProgramHeader%3CR,+Offset%3E\" class=\"impl\"><a class=\"src rightside\" href=\"src/gimli/read/line.rs.html#1022\">source</a><a href=\"#impl-StructuralPartialEq-for-LineProgramHeader%3CR,+Offset%3E\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;R, Offset&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/marker/trait.StructuralPartialEq.html\" title=\"trait core::marker::StructuralPartialEq\">StructuralPartialEq</a> for <a class=\"struct\" href=\"gimli/read/struct.LineProgramHeader.html\" title=\"struct gimli::read::LineProgramHeader\">LineProgramHeader</a>&lt;R, Offset&gt;<div class=\"where\">where\n    R: <a class=\"trait\" href=\"gimli/read/trait.Reader.html\" title=\"trait gimli::read::Reader\">Reader</a>&lt;Offset = Offset&gt;,\n    Offset: <a class=\"trait\" href=\"gimli/read/trait.ReaderOffset.html\" title=\"trait gimli::read::ReaderOffset\">ReaderOffset</a>,</div></h3></section>","StructuralPartialEq","gimli::read::line::LineNumberProgramHeader"]]
};if (window.register_type_impls) {window.register_type_impls(type_impls);} else {window.pending_type_impls = type_impls;}})()